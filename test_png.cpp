#include <iostream>
#include <stdexcept>
#include <functional>
#include <png.h>
#include <noise/noise.h>
#include <cstdio>
#include <utility>

using PixelGenerator = std::function<void(int x, int y, int& r, int& g, int& b)>;

class PngWriter {
public:
    PngWriter(const char* file_name, int width, int height, PixelGenerator pixel_generator);
    ~PngWriter();
    void write_image();
private:
    FILE* file;
    png_structp png_ptr;
    png_infop info_ptr;
    int width;
    int height;
    PixelGenerator pixel_generator;
    static void png_error(png_structp png_ptr, png_const_charp error_msg);
};

double linear_interpolate(double value, double low1, double high1, double low2, double high2);
void write_png_file(const char* file_name, int width, int height);


int main() {
    const char* file_name = "output.png";
    const int width = 800;
    const int height = 600;

    try {
        write_png_file(file_name, width, height);
    } catch (const std::exception& e) {
        std::cerr << "Error writing PNG file: " << e.what() << std::endl;
        return -1;
    }
}

void write_png_file(const char* file_name, int width, int height) {
    PixelGenerator pixel_generator = [&width](int x, int y, int& r, int& g, int& b) {
        noise::module::Perlin perlin_noise;

        perlin_noise.SetSeed(0);
        perlin_noise.SetFrequency(1.0 / width);
        perlin_noise.SetLacunarity(2);
        perlin_noise.SetOctaveCount(10);
        perlin_noise.SetNoiseQuality(noise::QUALITY_BEST);

        int value = static_cast<int>(linear_interpolate(
                perlin_noise.GetValue(x, y, 0),
                -1,
                1,
                0,
                255));

        r = g = b = value;
    };

    PngWriter png_writer(file_name, width, height, pixel_generator);
    png_writer.write_image();

}

PngWriter::PngWriter(const char* file_name, int width, int height, PixelGenerator pixel_generator) :
        file(fopen(file_name, "wb")),
        png_ptr(nullptr),
        info_ptr(nullptr),
        width(width),
        height(height),
        pixel_generator(std::move(pixel_generator)) {

    if (!file) {
        throw std::runtime_error("Failed to open file for writing");
    }

    png_ptr = png_create_write_struct(
            PNG_LIBPNG_VER_STRING,
            nullptr,
            PngWriter::png_error,
            nullptr);

    if (!png_ptr) {
        throw std::runtime_error("Failed to create PNG write struct");
    }

    info_ptr = png_create_info_struct(png_ptr);

    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, (png_infopp)nullptr);
        throw std::runtime_error("Failed to create PNG info struct");
    }

    png_init_io(png_ptr, file);

    png_set_IHDR(
            png_ptr, info_ptr,
            width, height,
            8,
            PNG_COLOR_TYPE_RGB,
            PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_BASE,
            PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);
}

PngWriter::~PngWriter() {
    if (png_ptr && info_ptr) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
    }

    if (file) fclose(file);
}

void PngWriter::write_image() {
    auto row = static_cast<png_bytep>(malloc(3 * width * sizeof(png_byte)));

    if (!row) {
        throw std::runtime_error("Failed to allocate row buffer");
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int r, g, b;
            pixel_generator(x, y, r, g, b);

            row[x * 3] = static_cast<png_byte>(r);
            row[x * 3 + 1] = static_cast<png_byte>(g);
            row[x * 3 + 2] = static_cast<png_byte>(b);
        }

        png_write_row(png_ptr, row);
    }

    png_write_end(png_ptr, nullptr);
    free(row);
}

void PngWriter::png_error(png_structp png_ptr [[maybe_unused]], png_const_charp error_msg) {
    throw std::runtime_error(error_msg);
}

double linear_interpolate(double value, double low1, double high1, double low2, double high2) {
    return low2 + (high2 - low2) * ((value - low1) / (high1 - low1));
}
