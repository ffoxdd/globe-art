#include <png.h>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <noise/noise.h>
#include <iostream>


int write_png_file(const char *filename, int width, int height);
void png_error(__attribute__((unused)) png_structp png_ptr, png_const_charp msg);
double linear_interpolate(double old_value, double old_min, double old_max, double new_min, double new_max);


int main() {
    write_png_file("output.png", 256, 256);
    return 0;
}

int write_png_file(const char *filename, int width, int height) {
    FILE *fp = fopen(filename, "wb");
    if (!fp) return -1;

    png_structp png_ptr = nullptr;
    png_infop info_ptr = nullptr;

    try {
        png_ptr = png_create_write_struct(
                PNG_LIBPNG_VER_STRING,
                nullptr,
                png_error,
                nullptr);
        if (!png_ptr) return -1;

        info_ptr = png_create_info_struct(png_ptr);
        if (!info_ptr) throw std::runtime_error("Failed to create PNG info struct");

        png_init_io(png_ptr, fp);

        png_set_IHDR(
                png_ptr, info_ptr,
                width, height,
                8,
                PNG_COLOR_TYPE_RGB,
                PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_BASE,
                PNG_FILTER_TYPE_BASE);

        png_write_info(png_ptr, info_ptr);

        /*************************************************************/

        noise::module::Perlin perlin_noise;

        perlin_noise.SetSeed(0);
        perlin_noise.SetFrequency(1.0 / width);
        perlin_noise.SetLacunarity(2);
        perlin_noise.SetOctaveCount(10);
        perlin_noise.SetNoiseQuality(noise::QUALITY_BEST);

        auto row = (png_bytep)malloc(3 * width * sizeof(png_byte));

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int value = (int)linear_interpolate(
                        perlin_noise.GetValue(x, y, 0),
                        -1,
                        1,
                        0,
                        255);

                row[x*3] = value;
                row[x*3 + 1] = value;
                row[x*3 + 2] = value;
            }

            png_write_row(png_ptr, row);
        }

        /*************************************************************/

        png_write_end(png_ptr, nullptr);

        // Cleanup.
        fclose(fp);
        png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        png_destroy_write_struct(&png_ptr, &info_ptr);
        if (row != nullptr) free(row);
    } catch (const std::runtime_error& e) {
        fclose(fp);
        if (png_ptr) png_destroy_write_struct(&png_ptr, &info_ptr); // Ensure png resources are freed on error
        return -1;
    }

    return 0;
}

void png_error(__attribute__((unused)) png_structp png_ptr, png_const_charp msg) {
    throw std::runtime_error(msg);
}

double linear_interpolate(double old_value, double old_min, double old_max, double new_min, double new_max) {
    if (old_min == old_max) {
        std::cerr << "Old range cannot be zero." << std::endl;
        return 0.0;
    }

    return ((old_value - old_min) * (new_max - new_min)) / (old_max - old_min) + new_min;
}
