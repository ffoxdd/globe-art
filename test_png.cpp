#include <png.h>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

void pngError(png_structp png_ptr, png_const_charp msg) {
    throw std::runtime_error(msg);
}

int write_png_file(const char *filename, int width, int height) {
    FILE *fp = fopen(filename, "wb");
    if (!fp) return -1;

    png_structp png_ptr = nullptr;
    png_infop info_ptr = nullptr;

    try {
        png_ptr = png_create_write_struct(
                PNG_LIBPNG_VER_STRING, nullptr, pngError, nullptr);
        if (!png_ptr) return -1;

        info_ptr = png_create_info_struct(png_ptr);
        if (!info_ptr) throw std::runtime_error("Failed to create PNG info struct");

        png_init_io(png_ptr, fp);

        // Set image attributes.
        png_set_IHDR(
                png_ptr, info_ptr, width, height,
                8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

        png_write_info(png_ptr, info_ptr);

        // Write the image data.
        auto row = (png_bytep)malloc(3 * width * sizeof(png_byte));
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                row[x*3] = x % 255; // Red
                row[x*3 + 1] = y % 255; // Green
                row[x*3 + 2] = (x * y) % 255; // Blue
            }
            png_write_row(png_ptr, row);
        }

        // Finish writing.
        png_write_end(png_ptr, nullptr);

        // Cleanup.
        fclose(fp);
        png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        png_destroy_write_struct(&png_ptr, &info_ptr);
        if (row != nullptr) free(row);
    } catch (const std::runtime_error& e) {
        // If an error occurs, clean up and return an error code.
        fclose(fp);
        if (png_ptr) png_destroy_write_struct(&png_ptr, &info_ptr); // Ensure png resources are freed on error
        return -1;
    }

    return 0;
}

int main() {
    write_png_file("output.png", 256, 256);
    return 0;
}
