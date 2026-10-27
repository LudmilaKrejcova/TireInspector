#define SDL_MAIN_HANDLED
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <SDL3/SDL.h>
#include <geGL/geGL.h>
#include <geGL/StaticCalls.h>
#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <cctype>
#include <sstream>

namespace {

    constexpr float pi = 3.14159265358979323846f;
    constexpr float worldScale = 0.01f;

    struct SdlManager {
        SDL_Window* window = nullptr;
        SDL_GLContext context = nullptr;

        SdlManager(const char* title, int width, int height) {
            if (!SDL_Init(SDL_INIT_VIDEO)) {
                throw std::runtime_error(std::string("Failed to initialize SDL3: ") + SDL_GetError());
            }
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);

            window = SDL_CreateWindow(title, width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
            if (!window) {
                SDL_Quit();
                throw std::runtime_error(std::string("Failed to create SDL window: ") + SDL_GetError());
            }
            context = SDL_GL_CreateContext(window);
            if (!context) {
                SDL_DestroyWindow(window);
                SDL_Quit();
                throw std::runtime_error(std::string("Failed to create OpenGL context: ") + SDL_GetError());
            }
            ge::gl::init();
            SDL_GL_SetSwapInterval(1);
        }

        ~SdlManager() {
            if (context) SDL_GL_DestroyContext(context);
            if (window) SDL_DestroyWindow(window);
            SDL_Quit();
        }
    };

    struct Options {
        std::string pathZ1;
        std::string pathZ2 = "none";
        std::string pathX = "auto";
        float meshJumpMm = 3.0f;
        float meshXJumpMm = 5.0f;
        std::string mode = "heat";
        int skipFactor = 2;
        int maxColumns = 768;
        int maxRows = 2048;
        int startRow = -1;
        int endRow = -1;
        bool flipZ2 = false;
        bool ignoreZ2 = false;
        float radiusMm = 320.0f;
        float zOffsetMm = -110.0f;
        float heightSign = -1.0f;
        float xStepMm = 0.2f;
        float pointSize = 2.0f;
        float rotationSpeedDeg = 12.0f;
        int xZeroCenterTolerancePx = 100;
        float zNeighborThresholdMm = 3.0f;
        float xNeighborThresholdMm = 5.0f;
    };

    int ceilDiv(int value, int divisor) {
        return (value + divisor - 1) / divisor;
    }

    bool isValidDepth(float value) {
        return std::isfinite(value) && value > 0.0f;
    }

    int parseInt(const char* value, const std::string& flag) {
        try {
            return std::stoi(value);
        }
        catch (...) {
            throw std::runtime_error("Invalid integer for " + flag + ": " + value);
        }
    }

    float parseFloat(const char* value, const std::string& flag) {
        try {
            const float result = std::stof(value);
            if (!std::isfinite(result)) throw std::invalid_argument("not finite");
            return result;
        }
        catch (...) {
            throw std::runtime_error("Invalid number for " + flag + ": " + value);
        }
    }

    Options parseOptions(int argc, char** argv) {
        Options options;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--flip") { options.flipZ2 = true; continue; }
            if (arg == "--hide-z2") { options.ignoreZ2 = true; continue; }
            if (arg == "--no-sidewalls") { continue; }
            if (arg == "--help") {
                std::cout
                    << "Tire Inspector - circular scan viewer\n"
                    << "  --z1 PATH             Primary Z scan\n"
                    << "  --z2 PATH             Secondary Z scan, or none\n"
                    << "  --x PATH              X coordinates, auto (default), or none\n"
                    << "  --mode single|heat|rg|r\n"
                    << "  --flip                Flip secondary Z horizontally\n"
                    << "  --hide-z2             Show only the primary scan\n"
                    << "  --radius MM           Approximate tire radius (default 320)\n"
                    << "  --z-offset MM         Primary Z reference offset (default -110)\n"
                    << "  --height-sign -1|1    Radial direction for Z differences (default -1)\n"
                    << "  --x-step MM           Fallback X pixel spacing (default 0.2)\n"
                    << "  --point-size PX       Point diameter in pixels (default 2)\n"
                    << "  --speed DEG_PER_SEC   Rotation speed (default 12)\n"
                    << "  --skip N              Minimum downsampling stride (default 2)\n"
                    << "  --max-columns N       Maximum scan columns (default 768)\n"
                    << "  --max-rows N          Maximum scan rows (default 2048)\n"
                    << "  --start-row N         First scan row, inclusive (default 0)\n"
                    << "  --end-row N           Last scan row, exclusive (default entire scan)\n"
                    << "  --no-sidewalls        Compatibility option; no synthetic geometry is drawn\n"
                    << "  --x-zero-center-px N  Half-width around source image center, for zero-X diagnostics (default 100)\n"
                    << "  --z-neighbor-mm MM    Local Z deviation threshold (default 3 mm)\n"
                    << "  --x-neighbor-mm MM    Local X deviation threshold (default 5 mm)\n"
                    << "Keys: P/Space play/pause, Arrow keys pan, +/- speed,\n"
                    << "      F side view, T tread view, R reset, M color mode,\n"
                    << "      H comparison visibility, G diagnostic colors, V perspective/orthographic,\n"
                    << "      [/] point size, Esc quit.\n"
                    << "Mouse: Left drag orbit, wheel zoom. W/S also zoom.\n";
                std::cout
                    << "New: start without arguments, then drop Z TIFFs or folders into the window.\n"
                    << "Matching X is found beside Z; --x none forces regular X spacing.\n"
                    << "--mesh-jump MM (default 3), --mesh-x-jump MM (default 5).\n"
                    << "B: points/surface/wireframe | Arrows: pan | Shift: faster\n"
                    << "A/D: orbit | Q/E: turn tire | G: diagnostics | V: projection\n"
                    << "O: open files | PageUp/PageDown: previous/next dataset | J: fast motion | Home: center\n"
                    << "Right/middle drag: pan | R: fit/reset | F/T: side/tread view\n";
                std::exit(0);
            }

            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + arg);
            const std::string value = argv[++i];
            if (arg == "--z1") options.pathZ1 = value;
            else if (arg == "--z2") options.pathZ2 = value;
            else if (arg == "--x") options.pathX = value;
            else if (arg == "--mode") options.mode = value;
            else if (arg == "--skip") options.skipFactor = parseInt(value.c_str(), arg);
            else if (arg == "--max-columns") options.maxColumns = parseInt(value.c_str(), arg);
            else if (arg == "--max-rows") options.maxRows = parseInt(value.c_str(), arg);
            else if (arg == "--start-row") options.startRow = parseInt(value.c_str(), arg);
            else if (arg == "--end-row") options.endRow = parseInt(value.c_str(), arg);
            else if (arg == "--radius") options.radiusMm = parseFloat(value.c_str(), arg);
            else if (arg == "--z-offset") options.zOffsetMm = parseFloat(value.c_str(), arg);
            else if (arg == "--height-sign") options.heightSign = parseFloat(value.c_str(), arg);
            else if (arg == "--x-step") options.xStepMm = parseFloat(value.c_str(), arg);
            else if (arg == "--point-size") options.pointSize = parseFloat(value.c_str(), arg);
            else if (arg == "--x-zero-center-px") options.xZeroCenterTolerancePx = parseInt(value.c_str(), arg);
            else if (arg == "--z-neighbor-mm") options.zNeighborThresholdMm = parseFloat(value.c_str(), arg);
            else if (arg == "--x-neighbor-mm") options.xNeighborThresholdMm = parseFloat(value.c_str(), arg);
            else if (arg == "--mesh-jump") options.meshJumpMm = parseFloat(value.c_str(), arg);
            else if (arg == "--mesh-x-jump") options.meshXJumpMm = parseFloat(value.c_str(), arg);
            else if (arg == "--jump" || arg == "--seam-jump") {
                std::cerr << "Notice: " << arg << " is a legacy option; use --mesh-jump and --mesh-x-jump instead.\n";
            }
            else if (arg == "--speed") options.rotationSpeedDeg = parseFloat(value.c_str(), arg);
            else if (arg == "--slope") {
                std::cerr << "Notice: --slope is a legacy option; use --mesh-jump and --mesh-x-jump instead.\n";
            }
            else throw std::runtime_error("Unknown argument: " + arg);
        }

        if (options.skipFactor < 1 || options.maxColumns < 2 || options.maxRows < 3 ||
            options.meshJumpMm <= 0.0f || options.meshXJumpMm <= 0.0f ||
            options.radiusMm <= 0.0f || options.xStepMm <= 0.0f ||
            options.pointSize <= 0.0f || options.pointSize > 64.0f ||
            options.rotationSpeedDeg < 0.0f || options.xZeroCenterTolerancePx < 0 ||
            options.zNeighborThresholdMm <= 0.0f || options.xNeighborThresholdMm <= 0.0f ||
            (options.heightSign != 1.0f && options.heightSign != -1.0f)) {
            throw std::runtime_error("Invalid geometry or sampling parameter. See --help.");
        }
        return options;
    }

    struct ScanGrid {
        cv::Mat ranger;
        cv::Mat ruler;
        cv::Mat x;
        cv::Mat localFlags;
        std::vector<uint8_t> valid;
        std::vector<GLuint> pointIndices;
        std::vector<GLuint> triangleIndices;
    };

    // BEGIN TESTABLE MESH HELPERS
    // Require support throughout the original cell, including samples omitted by downsampling.
    template<class Sample>
    bool cellIsSafe(int x0, int y0, int x1, int y1, Sample sample,
        float zJump, float xJump) {
        auto compatible = [=](const std::array<float, 2>& a, const std::array<float, 2>& b) {
            return std::fabs(a[0] - b[0]) <= zJump && std::fabs(a[1] - b[1]) <= xJump;
            };
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const auto value = sample(x, y);
                if (!std::isfinite(value[0]) || !std::isfinite(value[1])) return false;
                if (x > x0 && !compatible(value, sample(x - 1, y))) return false;
                if (y > y0 && !compatible(value, sample(x, y - 1))) return false;
            }
        }
        const auto a = sample(x0, y0), b = sample(x1, y0);
        const auto c = sample(x0, y1), d = sample(x1, y1);
        return compatible(a, b) && compatible(a, c) && compatible(b, d) &&
            compatible(c, d) && compatible(b, c);
    }

    template<class SafeCell>
    std::vector<uint32_t> makeTriangleIndices(int width, int height, SafeCell safeCell) {
        std::vector<uint32_t> indices;
        for (int y = 0; y + 1 < height; ++y) {
            for (int x = 0; x + 1 < width; ++x) {
                if (!safeCell(x, y)) continue;
                const uint32_t a = uint32_t(y * width + x);
                const uint32_t b = a + 1, c = a + uint32_t(width), d = c + 1;
                indices.insert(indices.end(), { a, b, c, b, d, c });
            }
        }
        // Do not bridge the first/last scan rows: the unobserved seam remains open.
        return indices;
    }
    // END TESTABLE MESH HELPERS

    std::vector<GLuint> buildSurfaceIndices(const cv::Mat& sourceRanger, const cv::Mat& sourceX,
        bool hasX, const Options& options, int width, int height, int columnStride, int rowStride, int startRow) {
        const int sourceWidth = sourceRanger.cols;
        auto originalSample = [&](int column, int row) -> std::array<float, 2> {
            const float z = sourceRanger.at<float>(row, column);
            const float x = hasX ? sourceX.at<float>(sourceX.rows == 1 ? 0 : row, column)
                : (float(column) - 0.5f * (sourceWidth - 1)) * options.xStepMm;
            const bool zeroPlaceholder = hasX && std::fabs(x) < 0.0001f &&
                std::fabs(float(column) - 0.5f * (sourceWidth - 1)) > options.xZeroCenterTolerancePx;
            const float radius = options.radiusMm + options.heightSign * (z + options.zOffsetMm);
            if (!isValidDepth(z) || zeroPlaceholder || radius <= 0.0f)
                return { std::numeric_limits<float>::quiet_NaN(), x };
            return { z, x };
            };
        return makeTriangleIndices(width, height, [&](int column, int row) {
            return cellIsSafe(column * columnStride, startRow + row * rowStride,
                (column + 1) * columnStride, startRow + (row + 1) * rowStride,
                originalSample, options.meshJumpMm, options.meshXJumpMm);
            });
    }

    ScanGrid buildScanGrid(const cv::Mat& sourceRanger,
        const cv::Mat& sourceRuler,
        const cv::Mat& sourceX,
        bool hasX,
        const Options& options,
        int textureLimit) {
        const int sourceWidth = sourceRanger.cols;
        const int sourceHeight = sourceRanger.rows;

        // Keep empty rows: trimming them would hide missing regions of the scan.
        const int startRow = options.startRow < 0 ? 0 : options.startRow;
        const int endRow = options.endRow < 0 ? sourceHeight : options.endRow;

        if (startRow < 0 || startRow >= sourceHeight || endRow > sourceHeight || endRow - startRow < 3) {
            throw std::runtime_error("Invalid scan row range.");
        }

        const int rowCount = endRow - startRow;
        const int maxColumns = std::min(options.maxColumns, textureLimit);
        const int maxRows = std::min(options.maxRows, textureLimit);
        const int columnStride = std::max(options.skipFactor, ceilDiv(sourceWidth, maxColumns));
        const int rowStride = std::max(options.skipFactor, ceilDiv(rowCount, maxRows));
        const int width = ceilDiv(sourceWidth, columnStride);
        const int height = ceilDiv(rowCount, rowStride);
        if (width < 2 || height < 3) {
            throw std::runtime_error("Not enough samples remain to display the scan.");
        }

        std::cout << "Source: " << sourceWidth << " x " << sourceHeight
            << ", using rows [" << startRow << ", " << endRow << ")\n"
            << "GPU grid: " << width << " x " << height
            << " (X stride " << columnStride << ", Y stride " << rowStride << ")\n";

        ScanGrid grid;
        grid.ranger.create(height, width, CV_32FC1);
        grid.ruler.create(height, width, CV_32FC1);
        grid.x.create(height, width, CV_32FC1);
        grid.localFlags = cv::Mat::zeros(height, width, CV_32FC1);
        grid.valid.resize(static_cast<size_t>(width) * height, 0);

        uint64_t validCount = 0;
        uint64_t suspiciousZeroXCount = 0;
        uint64_t centralZeroXCount = 0;
        uint64_t invertedRadiusCount = 0;
        uint64_t centralRadiusCount = 0;
        uint64_t invalidXCount = 0;
        uint64_t invalidZCount = 0;
        uint64_t localXCount = 0;
        uint64_t localZCount = 0;
        uint64_t isolatedCount = 0;
        float minZ = std::numeric_limits<float>::infinity();
        float maxZ = -std::numeric_limits<float>::infinity();

        for (int y = 0; y < height; ++y) {
            const int sourceY = startRow + y * rowStride;
            const float* rangerRow = sourceRanger.ptr<float>(sourceY);
            const float* rulerRow = sourceRuler.ptr<float>(sourceY);
            const float* xRow = hasX ? sourceX.ptr<float>(sourceX.rows == 1 ? 0 : sourceY) : nullptr;
            float* dstRanger = grid.ranger.ptr<float>(y);
            float* dstRuler = grid.ruler.ptr<float>(y);
            float* dstX = grid.x.ptr<float>(y);

            for (int x = 0; x < width; ++x) {
                const int sourceCol = x * columnStride;
                const int sourceColRuler = options.flipZ2 ? sourceWidth - 1 - sourceCol : sourceCol;
                const float z = rangerRow[sourceCol];
                const float zSecondary = rulerRow[sourceColRuler];
                const float realX = hasX
                    ? xRow[sourceCol]
                    : (static_cast<float>(sourceCol) - 0.5f * (sourceWidth - 1)) * options.xStepMm;

                dstRanger[x] = z;
                dstRuler[x] = zSecondary;
                dstX[x] = realX;

                // Preserve the supplied viewer's zero-placeholder filtering.
                const float centerDistance = std::fabs(float(sourceCol) - 0.5f * (sourceWidth - 1));
                const bool suspiciousZero = hasX && std::fabs(realX) < 0.0001f &&
                    centerDistance > options.xZeroCenterTolerancePx;
                const bool xValid = std::isfinite(realX) && !suspiciousZero;
                const bool valid = isValidDepth(z) && xValid;
                grid.valid[static_cast<size_t>(y) * width + x] = static_cast<uint8_t>(valid);
                if (valid) {
                    ++validCount;
                    minZ = std::min(minZ, z);
                    maxZ = std::max(maxZ, z);
                    const float computedRadiusMm = options.radiusMm +
                        options.heightSign * (z + options.zOffsetMm);
                    if (computedRadiusMm <= 0.0f) ++invertedRadiusCount;
                    if (computedRadiusMm < options.radiusMm * 0.5f) ++centralRadiusCount;
                    if (hasX && std::fabs(realX) < 0.0001f) {
                        const float distanceFromCenter = std::fabs(
                            static_cast<float>(sourceCol) - 0.5f * (sourceWidth - 1));
                        if (distanceFromCenter > options.xZeroCenterTolerancePx) {
                            ++suspiciousZeroXCount;
                        }
                        else {
                            ++centralZeroXCount;
                        }
                    }
                }
                else {
                    if (!isValidDepth(z)) ++invalidZCount;
                    if (!xValid) ++invalidXCount;
                }
            }
        }

        if (validCount < 1) {
            throw std::runtime_error("No valid scan points remain after sampling.");
        }

        // Inspect immediate neighbors in the ORIGINAL scan, not in the downsampled grid.
        // These flags are purely diagnostic and never remove or move measured points.
        // Bit 0: unusual X, bit 1: unusual Z, bit 2: little local support.
        for (int y = 0; y < height; ++y) {
            const int sourceY = startRow + y * rowStride;
            float* flagRow = grid.localFlags.ptr<float>(y);
            for (int x = 0; x < width; ++x) {
                if (!grid.valid[static_cast<size_t>(y) * width + x]) continue;

                const int sourceCol = x * columnStride;
                const float measuredZ = sourceRanger.ptr<float>(sourceY)[sourceCol];
                const float measuredX = grid.x.ptr<float>(y)[x];
                std::array<float, 8> neighborZ{};
                std::array<float, 8> neighborX{};
                int neighborCount = 0;

                for (int dy = -1; dy <= 1; ++dy) {
                    const int neighborY = sourceY + dy;
                    if (neighborY < startRow || neighborY >= endRow) continue;
                    const float* zRow = sourceRanger.ptr<float>(neighborY);
                    const float* xRow = hasX ? sourceX.ptr<float>(sourceX.rows == 1 ? 0 : neighborY) : nullptr;
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int neighborColumn = sourceCol + dx;
                        if (neighborColumn < 0 || neighborColumn >= sourceWidth) continue;
                        const float neighborDepth = zRow[neighborColumn];
                        const float neighborPositionX = hasX
                            ? xRow[neighborColumn]
                            : (static_cast<float>(neighborColumn) - 0.5f * (sourceWidth - 1)) * options.xStepMm;
                        if (!isValidDepth(neighborDepth) || !std::isfinite(neighborPositionX)) continue;
                        neighborZ[neighborCount] = neighborDepth;
                        neighborX[neighborCount] = neighborPositionX;
                        ++neighborCount;
                    }
                }

                int flags = 0;
                if (neighborCount < 2) {
                    flags |= 4;
                    ++isolatedCount;
                }
                if (neighborCount >= 4) {
                    auto medianOf = [neighborCount](std::array<float, 8>& values) {
                        const int middle = neighborCount / 2;
                        std::nth_element(values.begin(), values.begin() + middle,
                            values.begin() + neighborCount);
                        return values[middle];
                        };
                    const float medianZ = medianOf(neighborZ);
                    const float medianX = medianOf(neighborX);
                    if (std::fabs(measuredX - medianX) > options.xNeighborThresholdMm) {
                        flags |= 1;
                        ++localXCount;
                    }
                    if (std::fabs(measuredZ - medianZ) > options.zNeighborThresholdMm) {
                        flags |= 2;
                        ++localZCount;
                    }
                }
                flagRow[x] = static_cast<float>(flags);
            }
        }

        // Build only points here. Surface indices are generated on demand.
        grid.pointIndices.reserve(static_cast<size_t>(validCount));
        for (size_t index = 0; index < grid.valid.size(); ++index) {
            if (grid.valid[index]) grid.pointIndices.push_back(static_cast<GLuint>(index));
        }

        const double coverage = 100.0 * static_cast<double>(validCount) /
            static_cast<double>(grid.valid.size());
        std::cout << "Sampled valid points: " << validCount << " / " << grid.valid.size()
            << " (" << coverage << " % coverage at this sampling level)\n"
            << "Surface mode connects supported cells; no missing vertices are filled.\n"
            << "Valid Z range: [" << minZ << ", " << maxZ << "]\n"
            << "Invalid Z samples: " << invalidZCount
            << ", invalid X samples: " << invalidXCount << '\n'
            << "Computed radius <= 0: " << invertedRadiusCount << '\n'
            << "Computed radius below half the nominal tire radius: "
            << centralRadiusCount << '\n'
            << "Local X outliers (> " << options.xNeighborThresholdMm << " mm): " << localXCount << '\n'
            << "Local Z outliers (> " << options.zNeighborThresholdMm << " mm): " << localZCount << '\n'
            << "Isolated samples (< 2 valid neighbors): " << isolatedCount << '\n';
        if (hasX) {
            std::cout << "Zero-X samples near source center: " << centralZeroXCount << '\n'
                << "Zero-X samples far from center (possible placeholders): "
                << suspiciousZeroXCount << '\n';
        }
        else {
            std::cout << "Original X map is unavailable; zero-X diagnostic is not applicable.\n";
        }
        std::cout << "Press G for diagnostics. Invalid Z/X and distant zero-X placeholders are filtered.\n";
        return grid;
    }

    std::shared_ptr<ge::gl::Texture> uploadFloatTexture(const cv::Mat& data) {
        auto texture = std::make_shared<ge::gl::Texture>(
            GL_TEXTURE_2D, GL_R32F, 1, data.cols, data.rows);
        texture->texParameteri(GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        texture->texParameteri(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        texture->texParameteri(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        texture->texParameteri(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        ge::gl::glTextureSubImage2D(texture->getId(), 0, 0, 0,
            data.cols, data.rows, GL_RED, GL_FLOAT, data.ptr());
        return texture;
    }

    const char* scanVertexShader = R"GLSL(
#version 450 core
layout(binding = 0) uniform sampler2D mapRanger;
layout(binding = 1) uniform sampler2D mapRuler;
layout(binding = 2) uniform sampler2D mapX;
layout(binding = 3) uniform sampler2D mapLocalFlags;

uniform mat4 mvpMatrix;
uniform float tireRadiusMm;
uniform float zOffsetMm;
uniform float heightSign;
uniform float globalScale;
uniform float pointSize;
uniform int ignoreZ2;
uniform int sourceWidth;
uniform int columnStride;
uniform int rowStride;
uniform int startRow;
uniform int sourceHeight;
uniform int xZeroCenterTolerancePx;

out vec3 surfacePosition;
out float heightDifference;
out float suspectZeroX;
out float suspectSmallRadius;
flat out int localFlags;

void main() {
    ivec2 dimensions = textureSize(mapRanger, 0);
    int currentX = gl_VertexID % dimensions.x;
    int currentY = gl_VertexID / dimensions.x;
    ivec2 point = ivec2(currentX, currentY);

    float zRanger = texelFetch(mapRanger, point, 0).r;
    float zRuler = texelFetch(mapRuler, point, 0).r;
    float xMm = texelFetch(mapX, point, 0).r;
    localFlags = int(texelFetch(mapLocalFlags, point, 0).r + 0.5);

    float validRuler = (ignoreZ2 == 1 || isnan(zRuler) || isinf(zRuler) || zRuler <= 0.0)
        ? zRanger : zRuler;
    heightDifference = zRanger - validRuler;

    // One complete scan maps to exactly one revolution around the X axis.
    float angle = 6.283185307179586 * float(startRow + currentY * rowStride) / float(sourceHeight);
    float surfaceRadiusMm = tireRadiusMm + heightSign * (zRanger + zOffsetMm);
    float surfaceRadius = surfaceRadiusMm * globalScale;
    int originalColumn = currentX * columnStride;
    float centerDistance = abs(float(originalColumn) - 0.5 * float(sourceWidth - 1));
    suspectZeroX = abs(xMm) < 0.0001 &&
                   centerDistance > float(xZeroCenterTolerancePx) ? 1.0 : 0.0;
    suspectSmallRadius = surfaceRadiusMm < tireRadiusMm * 0.5 ? 1.0 : 0.0;
    vec3 localPos = vec3(xMm * globalScale,
                         sin(angle) * surfaceRadius,
                         cos(angle) * surfaceRadius);

    surfacePosition = localPos;
    gl_Position = mvpMatrix * vec4(localPos, 1.0);
    gl_PointSize = pointSize;
}
)GLSL";

    const char* scanFragmentShader = R"GLSL(
#version 450 core
in vec3 surfacePosition;
in float heightDifference;
in float suspectZeroX;
in float suspectSmallRadius;
flat in int localFlags;
out vec4 fragColor;

uniform int renderMode;
uniform int debugMode;
uniform int displayMode;
uniform int ignoreZ2;

void main() {
    // Make individual scan samples distinguishable without filling any gaps.
    vec2 pointOffset = gl_PointCoord - vec2(0.5);
    if (renderMode == 0 && dot(pointOffset, pointOffset) > 0.25) discard;

    if (debugMode == 2) {
        // Yellow: isolated point, red: local Z outlier, cyan: local X outlier.
        // Violet: both local X and Z outliers; gray: no local flag.
        if ((localFlags & 4) != 0) fragColor = vec4(1.0, 0.9, 0.0, 1.0);
        else if ((localFlags & 3) == 3) fragColor = vec4(0.9, 0.1, 0.9, 1.0);
        else if ((localFlags & 2) != 0) fragColor = vec4(1.0, 0.2, 0.1, 1.0);
        else if ((localFlags & 1) != 0) fragColor = vec4(0.1, 0.85, 1.0, 1.0);
        else fragColor = vec4(0.48, 0.51, 0.55, 1.0);
        return;
    }

    if (debugMode == 1) {
        // Magenta: X=0 away from the center of the original image.
        // Orange: computed tire radius is less than half the nominal radius.
        // Gray: all other valid, measured samples.
        if (suspectZeroX > 0.5) fragColor = vec4(1.0, 0.0, 0.8, 1.0);
        else if (suspectSmallRadius > 0.5) fragColor = vec4(1.0, 0.55, 0.0, 1.0);
        else fragColor = vec4(0.48, 0.51, 0.55, 1.0);
        return;
    }

    vec3 baseColor = vec3(0.78);
    vec3 finalColor = baseColor;
    int currentMode = ignoreZ2 == 1 ? 0 : displayMode;

    if (currentMode == 1) {
        float t = clamp(abs(heightDifference), 0.0, 1.0);
        vec3 c0 = vec3(0.3);
        vec3 c1 = vec3(0.0, 0.3, 1.0);
        vec3 c2 = vec3(0.0, 1.0, 0.0);
        vec3 c3 = vec3(1.0, 1.0, 0.0);
        vec3 c4 = vec3(1.0, 0.0, 0.0);
        if (t < 0.25) finalColor = mix(c0, c1, t * 4.0);
        else if (t < 0.5) finalColor = mix(c1, c2, (t - 0.25) * 4.0);
        else if (t < 0.75) finalColor = mix(c2, c3, (t - 0.5) * 4.0);
        else finalColor = mix(c3, c4, (t - 0.75) * 4.0);
    } else if (currentMode == 2 || currentMode == 3) {
        const float threshold = 0.02;
        if (heightDifference > threshold) {
            float strength = min((heightDifference - threshold) * 10.0, 1.0);
            finalColor = mix(baseColor, vec3(0.9, 0.1, 0.1), strength);
        } else if (currentMode == 2 && heightDifference < -threshold) {
            float strength = min((-heightDifference - threshold) * 10.0, 1.0);
            finalColor = mix(baseColor, vec3(0.0, 0.8, 0.0), strength);
        }
    }

    if (renderMode == 1) {
        vec3 normal = cross(dFdx(surfacePosition), dFdy(surfacePosition));
        float lengthSquared = dot(normal, normal);
        if (lengthSquared > 1e-16) {
            normal *= inversesqrt(lengthSquared);
            finalColor *= 0.45 + 0.55 * abs(dot(normal, normalize(vec3(0.4, -0.6, 0.7))));
        }
    }
    fragColor = vec4(finalColor, 1.0);
}
)GLSL";

    void setMatrix(GLuint program, const char* name, const glm::mat4& matrix) {
        ge::gl::glUniformMatrix4fv(ge::gl::glGetUniformLocation(program, name),
            1, GL_FALSE, glm::value_ptr(matrix));
    }

    void setFloat(GLuint program, const char* name, float value) {
        ge::gl::glUniform1f(ge::gl::glGetUniformLocation(program, name), value);
    }

    void setInt(GLuint program, const char* name, int value) {
        ge::gl::glUniform1i(ge::gl::glGetUniformLocation(program, name), value);
    }

} // namespace


namespace {
    namespace fs = std::filesystem;

    std::string lowerText(std::string text) {
        for (char& c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return text;
    }

    bool isZFile(const fs::path& path) {
        const std::string name = lowerText(path.filename().u8string());
        return (name.size() >= 6 && name.substr(name.size() - 6) == ".z.tif") ||
            (name.size() >= 7 && name.substr(name.size() - 7) == ".z.tiff");
    }

    std::string matchingX(const std::string& zPath) {
        const fs::path path = fs::u8path(zPath);
        if (!isZFile(path)) return "none";
        std::string expected = path.filename().u8string();
        expected[expected.size() - path.extension().u8string().size() - 1] = 'X';
        const fs::path parent = path.has_parent_path() ? path.parent_path() : fs::path(".");
        for (const auto& entry : fs::directory_iterator(parent)) {
            if (entry.is_regular_file() && lowerText(entry.path().filename().u8string()) == lowerText(expected))
                return entry.path().u8string();
        }
        return "none";
    }

    // Allocate once and read in blocks, retaining support for Windows Unicode paths.
    std::vector<unsigned char> readFileBytes(const std::string& name) {
        std::ifstream stream(fs::u8path(name), std::ios::binary | std::ios::ate);
        if (!stream) throw std::runtime_error("Cannot open: " + name);
        const auto length = stream.tellg();
        if (length <= 0) throw std::runtime_error("Empty or unreadable file: " + name);
        if (static_cast<uint64_t>(length) > std::numeric_limits<size_t>::max())
            throw std::runtime_error("File is too large for this process: " + name);
        std::vector<unsigned char> bytes(static_cast<size_t>(length));
        stream.seekg(0, std::ios::beg);
        constexpr size_t chunkSize = 8 * 1024 * 1024;
        for (size_t offset = 0; offset < bytes.size();) {
            const size_t count = std::min(chunkSize, bytes.size() - offset);
            if (!stream.read(reinterpret_cast<char*>(bytes.data() + offset), static_cast<std::streamsize>(count)))
                throw std::runtime_error("Incomplete file read: " + name);
            offset += count;
        }
        return bytes;
    }

    cv::Mat readFloatTiff(const std::string& name) {
        const auto bytes = readFileBytes(name);
        cv::Mat image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
        if (image.empty() || image.type() != CV_32FC1)
            throw std::runtime_error("Expected a single-channel float32 TIFF: " + name);
        return image;
    }

    struct Dataset {
        std::string z;
        std::string x = "auto";
        std::string secondary = "none";
        bool flipSecondary = false;
    };

    std::vector<std::string> discoverScans(const std::string& input) {
        const fs::path path = fs::u8path(input);
        std::vector<std::string> files;
        if (fs::is_directory(path)) {
            for (const auto& entry : fs::recursive_directory_iterator(path,
                fs::directory_options::skip_permission_denied)) {
                if (entry.is_regular_file() && isZFile(entry.path())) files.push_back(entry.path().u8string());
            }
            // Prefer fusion output when a directory also contains individual camera Z files.
            std::vector<fs::path> fusedFolders;
            for (const auto& file : files) {
                if (lowerText(fs::u8path(file).filename().u8string()).find("fused") != std::string::npos)
                    fusedFolders.push_back(fs::u8path(file).parent_path());
            }
            files.erase(std::remove_if(files.begin(), files.end(), [&](const std::string& file) {
                const fs::path p = fs::u8path(file);
                return lowerText(p.filename().u8string()).find("fused") == std::string::npos &&
                    std::find(fusedFolders.begin(), fusedFolders.end(), p.parent_path()) != fusedFolders.end();
                }), files.end());
        }
        else if (fs::is_regular_file(path)) {
            const std::string ext = lowerText(path.extension().u8string());
            if (ext != ".tif" && ext != ".tiff")
                throw std::runtime_error("Drop a Z TIFF or a folder containing *.Z.tif files.");
            const std::string stem = lowerText(path.stem().u8string());
            if ((stem.size() >= 2 && stem.substr(stem.size() - 2) == ".x") ||
                (stem.size() >= 4 && stem.substr(stem.size() - 4) == ".int"))
                throw std::runtime_error("Drop the Z file. Matching X is loaded automatically.");
            files.push_back(path.u8string());
        }
        else throw std::runtime_error("Path not found: " + input);
        std::sort(files.begin(), files.end());
        if (files.empty()) throw std::runtime_error("No *.Z.tif or *.Z.tiff files found in this folder.");
        for (auto& file : files) file = fs::weakly_canonical(fs::u8path(file)).u8string();
        return files;
    }

    struct Scene {
        ScanGrid grid;
        std::shared_ptr<ge::gl::Texture> zTexture, secondaryTexture, xTexture, flagsTexture;
        std::shared_ptr<ge::gl::VertexArray> pointsVao, meshVao, previewVao;
        std::shared_ptr<ge::gl::Buffer> pointsBuffer, meshBuffer, previewBuffer;
        int sourceWidth = 0, sourceHeight = 0, rowStride = 1, columnStride = 1, startRow = 0;
        bool hasSecondary = false, hasX = false, meshReady = false;
        GLsizei previewCount = 0;
        fs::file_time_type zModified{}, xModified{};
        uintmax_t zBytes = 0, xBytes = 0;
        std::string xPath;
        glm::vec3 center{ 0.0f };
        float boundRadius = 1.0f;
    };

    std::unique_ptr<Scene> loadScene(const Dataset& dataset, const Options& baseOptions, int textureLimit) {
        Options options = baseOptions;
        options.flipZ2 = dataset.flipSecondary;
        cv::Mat z = readFloatTiff(dataset.z);
        cv::Mat secondary = dataset.secondary == "none" ? z : readFloatTiff(dataset.secondary);
        if (secondary.size() != z.size()) throw std::runtime_error("Secondary Z dimensions do not match primary Z.");
        auto scene = std::make_unique<Scene>();
        scene->hasSecondary = dataset.secondary != "none";
        scene->xPath = dataset.x == "auto" ? matchingX(dataset.z) : dataset.x;
        cv::Mat x;
        if (scene->xPath != "none") {
            x = readFloatTiff(scene->xPath);
            if (x.cols != z.cols || (x.rows != 1 && x.rows != z.rows))
                throw std::runtime_error("Matching X dimensions do not match Z. Use --x none only to request regular spacing.");
            scene->hasX = true;
        }
        scene->zModified = fs::last_write_time(fs::u8path(dataset.z));
        scene->zBytes = fs::file_size(fs::u8path(dataset.z));
        if (scene->hasX) {
            scene->xModified = fs::last_write_time(fs::u8path(scene->xPath));
            scene->xBytes = fs::file_size(fs::u8path(scene->xPath));
        }
        scene->sourceWidth = z.cols;
        scene->sourceHeight = z.rows;
        scene->startRow = options.startRow < 0 ? 0 : options.startRow;
        const int endRow = options.endRow < 0 ? z.rows : options.endRow;
        scene->rowStride = std::max(options.skipFactor, ceilDiv(endRow - scene->startRow, std::min(options.maxRows, textureLimit)));
        scene->columnStride = std::max(options.skipFactor, ceilDiv(z.cols, std::min(options.maxColumns, textureLimit)));
        scene->grid = buildScanGrid(z, secondary, x, scene->hasX, options, textureLimit);
        scene->zTexture = uploadFloatTexture(scene->grid.ranger);
        scene->secondaryTexture = uploadFloatTexture(scene->grid.ruler);
        scene->xTexture = uploadFloatTexture(scene->grid.x);
        scene->flagsTexture = uploadFloatTexture(scene->grid.localFlags);
        scene->pointsVao = std::make_shared<ge::gl::VertexArray>();
        scene->meshVao = std::make_shared<ge::gl::VertexArray>();
        scene->pointsBuffer = std::make_shared<ge::gl::Buffer>(
            scene->grid.pointIndices.size() * sizeof(GLuint), scene->grid.pointIndices.data());
        scene->pointsVao->addElementBuffer(scene->pointsBuffer);
        std::vector<GLuint> previewIndices;
        // A 3x3 grid stride avoids bias from empty rows or irregular valid-point counts.
        for (int row = 0; row < scene->grid.ranger.rows; row += 3) {
            for (int col = 0; col < scene->grid.ranger.cols; col += 3) {
                const auto index = static_cast<GLuint>(row * scene->grid.ranger.cols + col);
                if (scene->grid.valid[index]) previewIndices.push_back(index);
            }
        }
        if (previewIndices.empty()) previewIndices.push_back(scene->grid.pointIndices.front());
        scene->previewCount = static_cast<GLsizei>(previewIndices.size());
        scene->previewVao = std::make_shared<ge::gl::VertexArray>();
        scene->previewBuffer = std::make_shared<ge::gl::Buffer>(
            previewIndices.size() * sizeof(GLuint), previewIndices.data());
        scene->previewVao->addElementBuffer(scene->previewBuffer);
        float minX = std::numeric_limits<float>::infinity(), maxX = -minX;
        for (GLuint index : scene->grid.pointIndices) {
            const float position = scene->grid.x.ptr<float>()[index] * worldScale;
            minX = std::min(minX, position); maxX = std::max(maxX, position);
        }
        scene->center = glm::vec3((minX + maxX) * 0.5f, 0.0f, 0.0f);
        scene->boundRadius = 0.001f;
        for (GLuint index : scene->grid.pointIndices) {
            const float dx = scene->grid.x.ptr<float>()[index] * worldScale - scene->center.x;
            const float radius = (options.radiusMm + options.heightSign *
                (scene->grid.ranger.ptr<float>()[index] + options.zOffsetMm)) * worldScale;
            scene->boundRadius = std::max(scene->boundRadius, std::sqrt(dx * dx + radius * radius));
        }
        return scene;
    }

    void prepareSurface(Scene& scene, const Dataset& dataset, const Options& options) {
        if (scene.meshReady) return;
        if (fs::last_write_time(fs::u8path(dataset.z)) != scene.zModified ||
            fs::file_size(fs::u8path(dataset.z)) != scene.zBytes ||
            (scene.hasX && (fs::last_write_time(fs::u8path(scene.xPath)) != scene.xModified ||
                fs::file_size(fs::u8path(scene.xPath)) != scene.xBytes)))
            throw std::runtime_error("Source files changed. Reopen the dataset before building its surface.");
        const cv::Mat z = readFloatTiff(dataset.z);
        const cv::Mat x = scene.hasX ? readFloatTiff(scene.xPath) : cv::Mat();
        if (z.cols != scene.sourceWidth || z.rows != scene.sourceHeight ||
            (scene.hasX && (x.cols != z.cols || (x.rows != 1 && x.rows != z.rows))))
            throw std::runtime_error("Source dimensions changed. Reopen the dataset.");
        auto indices = buildSurfaceIndices(z, x, scene.hasX, options, scene.grid.ranger.cols,
            scene.grid.ranger.rows, scene.columnStride, scene.rowStride, scene.startRow);
        if (!indices.empty()) {
            auto buffer = std::make_shared<ge::gl::Buffer>(indices.size() * sizeof(GLuint), indices.data());
            scene.meshVao->addElementBuffer(buffer);
            scene.meshBuffer = std::move(buffer);
        }
        scene.grid.triangleIndices = std::move(indices);
        scene.meshReady = true;
    }

    struct Camera {
        glm::vec3 target{ 0.0f };
        float distance = 10.0f, yaw = -0.65f, pitch = 0.35f;
        bool orthographic = false;
        glm::vec3 direction() const {
            return { std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch) };
        }
        glm::vec3 right() const { return glm::normalize(glm::cross(-direction(), glm::vec3(0, 0, 1))); }
        glm::vec3 up() const { return glm::normalize(glm::cross(right(), -direction())); }
        void zoom(float amount, float bound) {
            distance = std::clamp(distance * std::exp(amount), bound * 0.005f, bound * 1000.0f);
        }
        void orbit(float dx, float dy) {
            yaw = std::remainder(yaw + dx, 2.0f * pi);
            pitch = std::clamp(pitch + dy, -1.55f, 1.55f);
        }
        void fit(const Scene* scene, float aspect) {
            target = scene ? scene->center : glm::vec3(0);
            const float limitingHalfFov = std::min(glm::radians(22.5f), std::atan(std::tan(glm::radians(22.5f)) * aspect));
            distance = (scene ? scene->boundRadius : 3.2f) * 1.12f / std::sin(limitingHalfFov);
            yaw = -0.65f; pitch = 0.35f;
        }
    };

    struct DialogInbox {
        std::mutex mutex;
        bool busy = false, ready = false;
        std::vector<std::string> paths;
        std::string error;
    };
    DialogInbox dialogInbox;

    void SDLCALL dialogResult(void*, const char* const* fileList, int) {
        std::lock_guard<std::mutex> lock(dialogInbox.mutex);
        dialogInbox.paths.clear(); dialogInbox.error.clear();
        if (!fileList) dialogInbox.error = "File dialog failed. Drag files into the window instead.";
        else for (int i = 0; fileList[i]; ++i) dialogInbox.paths.emplace_back(fileList[i]);
        dialogInbox.busy = false; dialogInbox.ready = true;
    }

    void openDialog(SDL_Window* window, bool folder) {
        {
            std::lock_guard<std::mutex> lock(dialogInbox.mutex);
            if (dialogInbox.busy) return;
            dialogInbox.busy = true;
        }
        static const SDL_DialogFileFilter filters[] = { { "TIFF depth maps", "tif;tiff" } };
        if (folder) SDL_ShowOpenFolderDialog(dialogResult, nullptr, window, nullptr, false);
        else SDL_ShowOpenFileDialog(dialogResult, nullptr, window, filters, 1, nullptr, true);
    }

    constexpr int panelHeight = 234;
    struct Button { cv::Rect rect; std::string label; int action; };

    // OpenCV supplies a small raster UI, so no font package or GUI framework is required.
    class Panel {
        std::shared_ptr<ge::gl::Program> program;
        std::shared_ptr<ge::gl::VertexArray> vao;
        std::shared_ptr<ge::gl::Texture> texture;
        std::string previousText;
        int previousWidth = 0;
    public:
        std::vector<Button> buttons;
        Panel() {
            const char* vertex = R"GLSL(#version 450 core
out vec2 uv;
void main() {
    vec2 p = vec2((gl_VertexID == 1) ? 3.0 : -1.0, (gl_VertexID == 2) ? 3.0 : -1.0);
    gl_Position = vec4(p, 0, 1);
    uv = vec2((p.x + 1.0) * 0.5, 1.0 - (p.y + 1.0) * 0.5);
})GLSL";
            const char* fragment = R"GLSL(#version 450 core
layout(binding=4) uniform sampler2D panelImage;
in vec2 uv; out vec4 color;
void main() { color = texture(panelImage, uv); }
)GLSL";
            program = std::make_shared<ge::gl::Program>(
                std::make_shared<ge::gl::Shader>(GL_VERTEX_SHADER, vertex),
                std::make_shared<ge::gl::Shader>(GL_FRAGMENT_SHADER, fragment));
            vao = std::make_shared<ge::gl::VertexArray>();
        }
        int hit(float x, float y) const {
            for (const auto& button : buttons)
                if (button.rect.contains(cv::Point(int(x), int(y)))) return button.action;
            return -1;
        }
        void draw(int logicalWidth, int logicalHeight, int pixelWidth, int pixelHeight,
            const std::vector<std::string>& lines, const std::vector<std::string>& labels) {
            std::string key;
            for (const auto& line : lines) key += line + '\n';
            for (const auto& label : labels) key += label + '\n';
            if (key != previousText || logicalWidth != previousWidth) {
                cv::Mat image(panelHeight, logicalWidth, CV_8UC4, cv::Scalar(34, 28, 22, 255));
                auto text = [&](std::string value, int x, int y, double size, cv::Scalar color) {
                    // The window title retains the complete UTF-8 path; the built-in bitmap font is ASCII.
                    for (char& c : value) if (static_cast<unsigned char>(c) >= 128) c = '?';
                    const size_t maxChars = size_t(std::max(1, (logicalWidth - x - 10) / int(20 * size)));
                    if (value.size() > maxChars) value = value.substr(0, maxChars > 3 ? maxChars - 3 : 0) + "...";
                    cv::putText(image, value, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, size, color, 1, cv::LINE_AA);
                    };
                const cv::Scalar bright(242, 236, 226, 255), muted(190, 180, 160, 255);
                text(lines[0], 14, 24, 0.57, bright);
                text(lines[1], 14, 47, 0.46, muted);
                buttons.clear();
                const int buttonWidth = (logicalWidth - 28 - 5 * 8) / 6;
                for (size_t i = 0; i < labels.size(); ++i) {
                    const int row = int(i) / 6, col = int(i) % 6;
                    cv::Rect rect(14 + col * (buttonWidth + 8), 59 + row * 36, buttonWidth, 29);
                    buttons.push_back({ rect, labels[i], int(i) });
                    cv::rectangle(image, rect, cv::Scalar(72, 57, 40, 255), cv::FILLED);
                    text(labels[i], rect.x + 7, rect.y + 20, 0.43, bright);
                }
                text(lines[2], 14, 147, 0.45, cv::Scalar(123, 217, 246, 255));
                text(lines[3], 14, 169, 0.44, muted);
                text("Drag: orbit | Right drag / arrows: pan | Wheel / W,S: zoom | Shift: faster | R: fit", 14, 191, 0.43, muted);
                text("A,D: orbit | Q,E: turn tire | B: surface | G: diagnostics | PageUp/Down: dataset", 14, 213, 0.43, muted);
                if (!texture || previousWidth != logicalWidth) {
                    texture = std::make_shared<ge::gl::Texture>(GL_TEXTURE_2D, GL_RGBA8, 1, logicalWidth, panelHeight);
                    texture->texParameteri(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    texture->texParameteri(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    texture->texParameteri(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                    texture->texParameteri(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                }
                ge::gl::glTextureSubImage2D(texture->getId(), 0, 0, 0, logicalWidth, panelHeight,
                    GL_BGRA, GL_UNSIGNED_BYTE, image.ptr());
                previousText = key; previousWidth = logicalWidth;
            }
            const int height = int(std::lround(float(panelHeight) * pixelHeight / logicalHeight));
            ge::gl::glViewport(0, pixelHeight - height, pixelWidth, height);
            ge::gl::glDisable(GL_DEPTH_TEST);
            ge::gl::glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            program->use(); texture->bind(4); vao->bind();
            ge::gl::glDrawArrays(GL_TRIANGLES, 0, 3);
            ge::gl::glEnable(GL_DEPTH_TEST);
        }
    };
} // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        SdlManager sdlManager("Tire Inspector - drop Z TIFF files or a folder", 1280, 900);
        SDL_SetWindowMinimumSize(sdlManager.window, 1024, 620);
        GLint textureLimit = 0;
        ge::gl::glGetIntegerv(GL_MAX_TEXTURE_SIZE, &textureLimit);
        if (textureLimit < 3) throw std::runtime_error("Invalid OpenGL texture limit.");
        auto scanProgram = std::make_shared<ge::gl::Program>(
            std::make_shared<ge::gl::Shader>(GL_VERTEX_SHADER, scanVertexShader),
            std::make_shared<ge::gl::Shader>(GL_FRAGMENT_SHADER, scanFragmentShader));
        Panel panel;
        std::unique_ptr<Scene> scene;
        std::vector<Dataset> datasets;
        int activeIndex = -1, requestedIndex = -1;
        int displayMode = options.mode == "single" ? 0 : options.mode == "rg" ? 2 : options.mode == "r" ? 3 : 1;
        int renderMode = 0, debugMode = 0;
        bool ignoreZ2 = options.ignoreZ2, playing = false, running = true;
        Camera camera;
        float tireAngle = 0, pointSize = options.pointSize, speed = options.rotationSpeedDeg;
        bool fastMotion = true;
        bool movingPreview = false;
        Uint64 previewUntil = 0;
        int dragButton = 0;
        std::string status = "Drop a Z TIFF or folder, or click Open files / Open folder.";
        std::vector<std::string> droppedPaths;
        bool dropInProgress = false;

        auto addPaths = [&](const std::vector<std::string>& paths) {
            int firstIndex = -1;
            std::string lastError;
            for (const auto& input : paths) {
                try {
                    for (const auto& file : discoverScans(input)) {
                        auto found = std::find_if(datasets.begin(), datasets.end(), [&](const Dataset& d) {
                            return d.z == file;
                            });
                        int index;
                        if (found == datasets.end()) {
                            index = int(datasets.size()); datasets.push_back({ file });
                        }
                        else index = int(found - datasets.begin());
                        if (firstIndex < 0) firstIndex = index;
                    }
                }
                catch (const std::exception& error) {
                    lastError = error.what(); std::cerr << lastError << '\n';
                }
            }
            if (firstIndex >= 0) requestedIndex = firstIndex;
            status = lastError.empty() ? "Dataset list updated. PageUp/PageDown switches scans." : lastError;
            };
        if (!options.pathZ1.empty()) {
            try {
                if (fs::is_directory(fs::u8path(options.pathZ1))) addPaths({ options.pathZ1 });
                else {
                    datasets.push_back({ fs::weakly_canonical(fs::u8path(options.pathZ1)).u8string(),
                        options.pathX, options.pathZ2, options.flipZ2 });
                    requestedIndex = 0;
                }
            }
            catch (const std::exception& error) { status = error.what(); }
        }
        auto aspectRatio = [&]() {
            int w = 1, h = 1; SDL_GetWindowSize(sdlManager.window, &w, &h);
            return float(w) / float(std::max(1, h - panelHeight));
            };
        auto fit = [&]() { camera.fit(scene.get(), aspectRatio()); tireAngle = 0; playing = false; };
        auto navigate = [&](int direction) {
            if (datasets.empty()) return;
            const int current = requestedIndex >= 0 ? requestedIndex : activeIndex;
            requestedIndex = (std::max(0, current) + direction + int(datasets.size())) % int(datasets.size());
            };
        auto action = [&](int id) {
            switch (id) {
            case 0: openDialog(sdlManager.window, false); break;
            case 1: openDialog(sdlManager.window, true); break;
            case 2: navigate(-1); break;
            case 3: navigate(1); break;
            case 4: renderMode = (renderMode + 1) % 3; break;
            case 5: fit(); break;
            case 6: camera.orthographic = !camera.orthographic; break;
            case 7: displayMode = (displayMode + 1) % 4; break;
            case 8: if (scene && scene->hasSecondary) ignoreZ2 = !ignoreZ2; break;
            case 9: debugMode = (debugMode + 1) % 3; break;
            case 10: playing = !playing; break;
            case 11: fastMotion = !fastMotion; break;
            }
            };
        const std::array<std::string, 3> renderNames{ "Points", "Surface", "Wireframe" };
        const std::array<std::string, 4> colorNames{ "Single", "Heat", "Red/green", "Red" };
        const std::array<std::string, 3> diagnosticNames{ "Off", "Radius/X", "Local outliers" };
        auto drawPanel = [&]() {
            int w, h, pw, ph;
            SDL_GetWindowSize(sdlManager.window, &w, &h);
            SDL_GetWindowSizeInPixels(sdlManager.window, &pw, &ph);
            if (w <= 0 || pw <= 0 || ph <= 0) return;
            std::string title = "Tire Inspector | No scan loaded";
            std::string paths = "Select the Z file; matching X is detected in the same folder.";
            std::string stats = "Circular mapping assumes one full revolution in the original scan.";
            if (scene && activeIndex >= 0) {
                const Dataset& d = datasets[activeIndex];
                const fs::path p = fs::u8path(d.z);
                title = "[" + std::to_string(activeIndex + 1) + "/" + std::to_string(datasets.size()) + "] " +
                    p.parent_path().filename().u8string() + " / " + p.filename().u8string();
                paths = "X: " + (scene->hasX ? fs::u8path(scene->xPath).filename().u8string() :
                    "MISSING - regular spacing " + std::to_string(options.xStepMm) + " mm") +
                    " | Z2: " + (scene->hasSecondary ? fs::u8path(d.secondary).filename().u8string() : "none");
                stats = std::to_string(scene->grid.pointIndices.size()) + " points | " +
                    (scene->meshReady ? std::to_string(scene->grid.triangleIndices.size() / 3) + " triangles" : "surface not built") + " | " + (movingPreview ? "Motion preview" : renderNames[renderMode]) +
                    " | Diagnostics: " + diagnosticNames[debugMode] + " | Speed: " + std::to_string(int(speed)) + " deg/s";
                if (renderMode != 0 && scene->meshReady && scene->grid.triangleIndices.empty())
                    stats += " | No safe triangles: switch to Points";
            }
            panel.draw(w, h, pw, ph, { title, paths, status, stats },
                { "Open files [O]", "Open folder", "Previous [PgUp]", "Next [PgDn]", renderNames[renderMode] + " [B]", "Fit / reset [R]",
                 camera.orthographic ? "Orthographic [V]" : "Perspective [V]", "Color: " + colorNames[(!scene || ignoreZ2) ? 0 : displayMode],
                 !scene || !scene->hasSecondary ? "Compare: N/A" : ignoreZ2 ? "Compare: off" : "Compare: on",
                 "Diagnostics [G]", playing ? "Pause [Space]" : "Play [Space]", fastMotion ? "Fast motion: on [J]" : "Fast motion: off [J]" });
            };

        ge::gl::glEnable(GL_DEPTH_TEST);
        ge::gl::glEnable(GL_PROGRAM_POINT_SIZE);
        ge::gl::glDisable(GL_CULL_FACE);
        ge::gl::glClearColor(0.075f, 0.105f, 0.14f, 1.0f);
        auto previousFrame = std::chrono::steady_clock::now();
        while (running) {
            const auto currentFrame = std::chrono::steady_clock::now();
            const float dt = std::min(std::chrono::duration<float>(currentFrame - previousFrame).count(), 0.05f);
            previousFrame = currentFrame;
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_EVENT_QUIT) running = false;
                else if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) {
                    dragButton = 0; SDL_CaptureMouse(false);
                }
                else if (event.type == SDL_EVENT_DROP_BEGIN) {
                    dropInProgress = true; droppedPaths.clear();
                }
                else if (event.type == SDL_EVENT_DROP_FILE && event.drop.data) {
                    droppedPaths.emplace_back(event.drop.data);
                }
                else if (event.type == SDL_EVENT_DROP_COMPLETE) dropInProgress = false;
                else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                    if (event.button.y < panelHeight) {
                        if (event.button.button == SDL_BUTTON_LEFT) action(panel.hit(event.button.x, event.button.y));
                    }
                    else {
                        dragButton = event.button.button;
                        SDL_CaptureMouse(true);
                    }
                }
                else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                    if (event.button.button == dragButton) { dragButton = 0; SDL_CaptureMouse(false); }
                }
                else if (event.type == SDL_EVENT_MOUSE_MOTION && dragButton) {
                    if (dragButton == SDL_BUTTON_LEFT) camera.orbit(-event.motion.xrel * 0.005f, event.motion.yrel * 0.005f);
                    else if (dragButton == SDL_BUTTON_RIGHT || dragButton == SDL_BUTTON_MIDDLE) {
                        int w, h; SDL_GetWindowSize(sdlManager.window, &w, &h);
                        const float unitsPerPixel = 2 * camera.distance * std::tan(glm::radians(22.5f)) / std::max(1, h - panelHeight);
                        camera.target += (-event.motion.xrel * camera.right() + event.motion.yrel * camera.up()) * unitsPerPixel;
                    }
                }
                else if (event.type == SDL_EVENT_MOUSE_WHEEL) {
                    float mx, my; SDL_GetMouseState(&mx, &my);
                    if (my >= panelHeight) {
                        previewUntil = SDL_GetTicks() + 150;
                        const float sign = event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -1.0f : 1.0f;
                        camera.zoom(-event.wheel.y * sign * 0.12f, scene ? scene->boundRadius : 3.2f);
                    }
                }
                else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                    switch (event.key.key) {
                    case SDLK_ESCAPE: running = false; break;
                    case SDLK_J: action(11); break;
                    case SDLK_HOME: camera.target = scene ? scene->center : glm::vec3(0); break;
                    case SDLK_O: action(0); break;
                    case SDLK_PAGEUP: action(2); break;
                    case SDLK_PAGEDOWN: action(3); break;
                    case SDLK_B: action(4); break;
                    case SDLK_R: action(5); break;
                    case SDLK_V: action(6); break;
                    case SDLK_M: action(7); break;
                    case SDLK_H: action(8); break;
                    case SDLK_G: action(9); break;
                    case SDLK_P: case SDLK_SPACE: action(10); break;
                    case SDLK_F: camera.yaw = 0; camera.pitch = 0; break;
                    case SDLK_T: camera.yaw = -pi / 2; camera.pitch = 0; break;
                    case SDLK_LEFTBRACKET: pointSize = std::max(1.0f, pointSize - 0.5f); break;
                    case SDLK_RIGHTBRACKET: pointSize = std::min(20.0f, pointSize + 0.5f); break;
                    case SDLK_PLUS: case SDLK_EQUALS: case SDLK_KP_PLUS: speed = std::min(180.0f, speed + 3); break;
                    case SDLK_MINUS: case SDLK_KP_MINUS: speed = std::max(0.0f, speed - 3); break;
                    }
                }
            }
            if (!running) break;
            {
                std::vector<std::string> paths;
                {
                    std::lock_guard<std::mutex> lock(dialogInbox.mutex);
                    if (dialogInbox.ready) {
                        paths.swap(dialogInbox.paths);
                        if (!dialogInbox.error.empty()) status = dialogInbox.error;
                        dialogInbox.ready = false;
                    }
                }
                if (!paths.empty()) addPaths(paths);
            }
            if (!dropInProgress && !droppedPaths.empty()) { addPaths(droppedPaths); droppedPaths.clear(); }
            if (requestedIndex >= 0) {
                const int selected = requestedIndex; requestedIndex = -1;
                status = "Loading " + fs::u8path(datasets[selected].z).filename().u8string() + " ...";
                ge::gl::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                drawPanel(); SDL_GL_SwapWindow(sdlManager.window);
                try {
                    // Commit only after the entire new scene has loaded successfully.
                    const auto loadStarted = std::chrono::steady_clock::now();
                    auto nextScene = loadScene(datasets[selected], options, textureLimit);
                    std::cout << "Point scene loading: " << std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - loadStarted).count() << " seconds\n";
                    scene = std::move(nextScene); activeIndex = selected;
                    ignoreZ2 = options.ignoreZ2 || !scene->hasSecondary;
                    fit();
                    status = scene->hasX ? "Ready. Full original scan = one revolution; cropped rows keep their original angles."
                        : "WARNING: No matching X map; using regular X spacing.";
                    const std::string title = "Tire Inspector | " + datasets[selected].z;
                    SDL_SetWindowTitle(sdlManager.window, title.c_str());
                }
                catch (const std::exception& error) {
                    status = std::string("Load failed: ") + error.what();
                    std::cerr << status << '\n';
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Cannot load scan", status.c_str(), sdlManager.window);
                }
                previousFrame = std::chrono::steady_clock::now();
            }
            if (scene && renderMode != 0 && !scene->meshReady) {
                status = "Building surface for the first time ...";
                drawPanel(); SDL_GL_SwapWindow(sdlManager.window);
                const auto started = std::chrono::steady_clock::now();
                try {
                    prepareSurface(*scene, datasets[activeIndex], options);
                    status = "Surface ready. Reused until another dataset is loaded.";
                }
                catch (const std::exception& error) {
                    status = std::string("Surface failed: ") + error.what();
                    std::cerr << status << '\n'; renderMode = 0;
                }
                std::cout << "Surface preparation: " << std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - started).count() << " seconds\n";
                previousFrame = std::chrono::steady_clock::now();
            }
            const bool* keys = SDL_GetKeyboardState(nullptr);
            if (SDL_GetWindowFlags(sdlManager.window) & SDL_WINDOW_INPUT_FOCUS) {
                const float boost = keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT] ? 3.0f : 1.0f;
                const float panSpeed = camera.distance * 0.6f * dt * boost;
                camera.target += camera.right() * (float(keys[SDL_SCANCODE_RIGHT]) - float(keys[SDL_SCANCODE_LEFT])) * panSpeed;
                camera.target += camera.up() * (float(keys[SDL_SCANCODE_UP]) - float(keys[SDL_SCANCODE_DOWN])) * panSpeed;
                camera.orbit((float(keys[SDL_SCANCODE_D]) - float(keys[SDL_SCANCODE_A])) * dt * boost,
                    (float(keys[SDL_SCANCODE_I]) - float(keys[SDL_SCANCODE_K])) * dt * boost);
                camera.zoom((float(keys[SDL_SCANCODE_S]) - float(keys[SDL_SCANCODE_W])) * dt * boost,
                    scene ? scene->boundRadius : 3.2f);
                tireAngle += (float(keys[SDL_SCANCODE_E]) - float(keys[SDL_SCANCODE_Q])) * dt * boost;
            }
            const bool focused = (SDL_GetWindowFlags(sdlManager.window) & SDL_WINDOW_INPUT_FOCUS) != 0;
            const bool keyMotion = focused && (keys[SDL_SCANCODE_LEFT] || keys[SDL_SCANCODE_RIGHT] ||
                keys[SDL_SCANCODE_UP] || keys[SDL_SCANCODE_DOWN] || keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_D] ||
                keys[SDL_SCANCODE_I] || keys[SDL_SCANCODE_K] || keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_S] ||
                keys[SDL_SCANCODE_Q] || keys[SDL_SCANCODE_E]);
            movingPreview = fastMotion && (dragButton != 0 || keyMotion || playing || SDL_GetTicks() < previewUntil);
            if (playing) tireAngle += glm::radians(speed) * dt;
            tireAngle = std::remainder(tireAngle, 2 * pi);
            int w, h, pw, ph;
            SDL_GetWindowSize(sdlManager.window, &w, &h);
            SDL_GetWindowSizeInPixels(sdlManager.window, &pw, &ph);
            if (w <= 0 || h <= panelHeight || pw <= 0 || ph <= 0) { SDL_Delay(20); continue; }
            const int sceneHeight = ph - int(std::lround(float(panelHeight) * ph / h));
            if (sceneHeight <= 0) continue;
            ge::gl::glViewport(0, 0, pw, sceneHeight);
            ge::gl::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if (scene) {
                const float aspect = float(pw) / sceneHeight;
                const float bound = scene->boundRadius;
                const float farPlane = std::max(camera.distance + glm::length(camera.target - scene->center) + 4 * bound, 10 * bound);
                const float nearPlane = std::max(bound * 0.00001f, camera.distance * 0.0001f);
                glm::mat4 projection;
                if (camera.orthographic) {
                    const float halfHeight = camera.distance * std::tan(glm::radians(22.5f));
                    projection = glm::ortho(-halfHeight * aspect, halfHeight * aspect, -halfHeight, halfHeight, -farPlane, farPlane);
                }
                else projection = glm::perspective(glm::radians(45.0f), aspect, nearPlane, farPlane);
                const glm::mat4 view = glm::lookAt(camera.target + camera.direction() * camera.distance,
                    camera.target, glm::vec3(0, 0, 1));
                const glm::mat4 model = glm::rotate(glm::mat4(1), tireAngle, glm::vec3(1, 0, 0));
                scanProgram->use();
                const GLuint id = scanProgram->getId();
                scene->zTexture->bind(0); scene->secondaryTexture->bind(1);
                scene->xTexture->bind(2); scene->flagsTexture->bind(3);
                setMatrix(id, "mvpMatrix", projection * view * model);
                setFloat(id, "globalScale", worldScale);
                setFloat(id, "tireRadiusMm", options.radiusMm);
                setFloat(id, "zOffsetMm", options.zOffsetMm);
                setFloat(id, "heightSign", options.heightSign);
                setFloat(id, "pointSize", pointSize);
                setInt(id, "displayMode", displayMode); setInt(id, "debugMode", debugMode);
                setInt(id, "renderMode", movingPreview ? 0 : renderMode); setInt(id, "ignoreZ2", ignoreZ2 ? 1 : 0);
                setInt(id, "sourceWidth", scene->sourceWidth); setInt(id, "sourceHeight", scene->sourceHeight);
                setInt(id, "columnStride", scene->columnStride); setInt(id, "rowStride", scene->rowStride);
                setInt(id, "startRow", scene->startRow);
                setInt(id, "xZeroCenterTolerancePx", options.xZeroCenterTolerancePx);
                if (movingPreview) {
                    scene->previewVao->bind();
                    ge::gl::glDrawElements(GL_POINTS, scene->previewCount, GL_UNSIGNED_INT, nullptr);
                }
                else if (renderMode == 0) {
                    scene->pointsVao->bind();
                    ge::gl::glDrawElements(GL_POINTS, GLsizei(scene->grid.pointIndices.size()), GL_UNSIGNED_INT, nullptr);
                }
                else if (!scene->grid.triangleIndices.empty()) {
                    scene->meshVao->bind();
                    ge::gl::glPolygonMode(GL_FRONT_AND_BACK, renderMode == 2 ? GL_LINE : GL_FILL);
                    ge::gl::glDrawElements(GL_TRIANGLES, GLsizei(scene->grid.triangleIndices.size()), GL_UNSIGNED_INT, nullptr);
                    ge::gl::glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                }
            }
            drawPanel();
            SDL_GL_SwapWindow(sdlManager.window);
        }
    }
    catch (const std::exception& error) {
        std::cerr << "Fatal error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
