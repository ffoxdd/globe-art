#define ANL_IMPLEMENTATION
#include <anl/anl.h>

#define STB_IMAGE_IMPLEMENTATION
#include <anl/Imaging/stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <anl/Imaging/stb_image_write.h>

#include <iostream>
#include <vector>
#include <stdexcept>
#include <algorithm>

void saveImageToPNG(anl::CArray2Dd &image, const std::string &filename);

int main() {
    const int width = 512;
    const int height = 512;
    const double persistence = 0.5;
    const double lacunarity = 2.0;
    const double octaves = 2;
    const double frequency = 1.0;
    const std::string filename = "fractal_noise.png";

    anl::CKernel kernel;
    auto seed = kernel.constant(1546);

    auto fractal_noise = kernel.fractal(
        seed,
        kernel.simplexBasis(seed),
        kernel.constant(persistence),
        kernel.constant(lacunarity),
        kernel.constant(octaves),
        kernel.constant(frequency)
    );

    anl::CArray2Dd image(width, height);

    anl::map2DNoZ(
        anl::SEAMLESS_NONE,
        image,
        kernel,
        anl::SMappingRanges(),
        fractal_noise
    );

    image.scaleToRange(0, 127);

    try {
        saveImageToPNG(image, "fractal_noise.png");
        std::cout << "Image saved as " << filename << std::endl;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}

void saveImageToPNG(anl::CArray2Dd &image, const std::string &filename) {
    int width = image.width();
    int height = image.height();
    double *data = image.getData();
    std::vector<unsigned char> pixels(width * height);

    std::transform(
        data, data + (width * height), pixels.begin(),
        [](double value) { return static_cast<unsigned char>(value); }
    );

    int success = stbi_write_png(
        filename.c_str(),
        width,
        height,
        1,
        pixels.data(),
        width
    );

    if (success == 0) {
        throw std::runtime_error("Failed to save the image");
    }
}
