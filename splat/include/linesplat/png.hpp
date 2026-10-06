// A dependency-free PNG writer for previews. It stores the pixels without
// compression (deflate "stored" blocks), which every viewer reads; run the
// file through any optimizer if size matters.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace linesplat {

// rgb: height rows of width * 3 bytes.
void write_png_rgb(const std::string& path, int width, int height, const std::vector<uint8_t>& rgb);

}  // namespace linesplat
