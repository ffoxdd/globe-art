#define ANL_IMPLEMENTATION
#include <anl/anl.h>

#define STB_IMAGE_IMPLEMENTATION
#include <anl/Imaging/stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <anl/Imaging/stb_image_write.h>

#include <iostream>
#include <vector>

int main() {
    const int width = 512;
    const int height = 512;
    const int seed = 1546;

    anl::CKernel kernel;
    anl::CNoiseExecutor vm(kernel);

    auto simplexNoise = kernel.simplexBasis(kernel.seed(seed));

    anl::CArray2Dd image(width, height);

    anl::SMappingRanges ranges;
    ranges.mapx0 = 0.0;
    ranges.mapy0 = 0.0;
    ranges.mapx1 = 1.0;
    ranges.mapy1 = 1.0;

    anl::map2DNoZ(anl::SEAMLESS_NONE, image, kernel, ranges, simplexNoise);
    image.scaleToRange(0, 127);

    std::vector<unsigned char> imgData(width * height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double value = image.get(x, y);
            auto pixelValue = static_cast<unsigned char>(value);
            imgData[y * width + x] = pixelValue;
        }
    }

    // Save the image as a PNG file
    if (stbi_write_png("simplex_noise.png", width, height, 1, imgData.data(), width) == 0) {
        std::cerr << "Failed to save the image" << std::endl;
        return 1;
    }

    std::cout << "Image saved as simplex_noise.png" << std::endl;

    return 0;
}
