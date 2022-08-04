/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/gui.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_scheduler_init.h>
#include <filesystem/resolver.h>
#include <thread>
#include <nori/warp.h>




using namespace nori;

static int threadCount = -1;
static bool gui = true;

static void renderBlock(const Scene *scene, Sampler *sampler, ImageBlock &block)
{
    const Camera *camera = scene->getCamera();
    const Integrator *integrator = scene->getIntegrator();
    // PropertyList pl{};
    // PathTracerRecursive integrator{pl};

    size_t N = sampler->getSampleCount();
    
    Point2i offset = block.getOffset();
    Vector2i size  = block.getSize();

    /* Clear the block contents */
    block.clear();


    /* For each pixel and pixel sample sample */
    for (int y=0; y<size.y(); ++y)
    {
        for (int x=0; x<size.x(); ++x)
        {
            Ray3f ray{};
            Color3f color{0.f};
            Point2f pixelSample = Point2f(float(x + offset.x()), float(y + offset.y())) + sampler->next2D();  // go through the centre of the pixel
            for (size_t i=0; i<N; ++i) // N = Target sample count per pixel
            {
                pixelSample.x() = std::min(pixelSample.x(), std::nextafter(float(x + offset.x() + 1), 0.f));
                pixelSample.y() = std::min(pixelSample.y(), std::nextafter(float(y + offset.y() + 1), 0.f));
                Point2f apertureSample = sampler->next2D();
                camera->sampleRay(ray, pixelSample, apertureSample);
                color += integrator->Li(scene, sampler, ray);
            }
            block.put(pixelSample, color / N);
        }
    }
}

static void render(Scene *scene, const std::string &filename) {
    const Camera *camera = scene->getCamera();
    Vector2i outputSize = camera->getOutputSize();
    scene->getIntegrator()->preprocess(scene);

    /* Create a block generator (i.e. a work scheduler) */
    BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);

    /* Allocate memory for the entire output image and clear it */
    ImageBlock result(outputSize, camera->getReconstructionFilter());
    result.clear();

    /* Create a window that visualizes the partially rendered result */
    NoriScreen *screen = nullptr;
    if (gui) {
        nanogui::init();
        screen = new NoriScreen(result);
    }

    /* Do the following in parallel and asynchronously */
    std::thread render_thread([&] {
        tbb::task_scheduler_init init(threadCount);

        cout << "Rendering .. ";
        cout.flush();
        auto before = std::chrono::system_clock::now();
        Timer timer;

        tbb::blocked_range<int> range(0, blockGenerator.getBlockCount());

        auto map = [&](const tbb::blocked_range<int> &range) {
            /* Allocate memory for a small image block to be rendered
               by the current thread */
            ImageBlock block(Vector2i(NORI_BLOCK_SIZE),
                camera->getReconstructionFilter());

            /* Create a clone of the sampler for the current thread */
            std::unique_ptr<Sampler> sampler(scene->getSampler()->clone());

            for (int i=range.begin(); i<range.end(); ++i) {
                /* Request an image block from the block generator */
                blockGenerator.next(block);

                /* Inform the sampler about the block to be rendered */
                sampler->prepare(block);

                /* Render all contained pixels */
                renderBlock(scene, sampler.get(), block);

                /* The image block has been processed. Now add it to
                   the "big" block that represents the entire image */
                result.put(block);
            }
        };

        /// Default: parallel rendering
        tbb::parallel_for(range, map);

        /// (equivalent to the following single-threaded call)
        // map(range);

        cout << "done. (took " << timer.elapsedString() << ")" << endl;
        auto after = std::chrono::system_clock::now();
        std::cout << "# benchmark # Rendering took: " << std::chrono::duration<double>(after - before).count() << " s" << std::endl;
    });

    /* Enter the application main loop */
    if (gui)
        nanogui::mainloop(50.f);

    /* Shut down the user interface */
    render_thread.join();

    if (gui) {
        delete screen;
        nanogui::shutdown();
    }

    /* Now turn the rendered image block into
       a properly normalized bitmap */
    std::unique_ptr<Bitmap> bitmap(result.toBitmap());

    /* Determine the filename of the output bitmap */
    std::string outputName = filename;
    size_t lastdot = outputName.find_last_of(".");
    if (lastdot != std::string::npos)
        outputName.erase(lastdot, std::string::npos);

    /* Save using the OpenEXR format */
    bitmap->saveEXR(outputName);

    /* Save tonemapped (sRGB) output using the PNG format */
    bitmap->savePNG(outputName);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        cerr << "Syntax: " << argv[0] << " <scene.xml> [--no-gui] [--threads N]" <<  endl;
        return -1;
    }
    
    std::string sceneName = "";
    std::string exrName = "";

    for (int i = 1; i < argc; ++i) {
        std::string token(argv[i]);
        if (token == "-t" || token == "--threads") {
            if (i+1 >= argc) {
                cerr << "\"--threads\" argument expects a positive integer following it." << endl;
                return -1;
            }
            threadCount = atoi(argv[i+1]);
            i++;
            if (threadCount <= 0) {
                cerr << "\"--threads\" argument expects a positive integer following it." << endl;
                return -1;
            }

            continue;
        }
        else if (token == "--no-gui") {
            gui = false;
            continue;
        }

        filesystem::path path(argv[i]);

        try {
            if (path.extension() == "xml") {
                sceneName = argv[i];

                /* Add the parent directory of the scene file to the
                   file resolver. That way, the XML file can reference
                   resources (OBJ files, textures) using relative paths */
                getFileResolver()->prepend(path.parent_path());
            } else if (path.extension() == "exr") {
                /* Alternatively, provide a basic OpenEXR image viewer */
                exrName = argv[i];
            } else {
                cerr << "Fatal error: unknown file \"" << argv[i]
                     << "\", expected an extension of type .xml or .exr" << endl;
            }
        } catch (const std::exception &e) {
            cerr << "Fatal error: " << e.what() << endl;
            return -1;
        }
    }

    if (exrName !="" && sceneName !="") {
        cerr << "Both .xml and .exr files were provided. Please only provide one of them." << endl;
        return -1;
    }
    else if (exrName == "" && sceneName == "") {
        cerr << "Please provide the path to a .xml (or .exr) file." << endl;
        return -1;
    }
    else if (exrName != "") {
        if (!gui) {
            cerr << "Flag --no-gui was set. Please remove it to display the EXR file." << endl;
            return -1;
        }
        try {
            Bitmap bitmap(exrName);
            ImageBlock block(Vector2i((int) bitmap.cols(), (int) bitmap.rows()), nullptr);
            block.fromBitmap(bitmap);
            nanogui::init();
            NoriScreen *screen = new NoriScreen(block);
            nanogui::mainloop(50.f);
            delete screen;
            nanogui::shutdown();
        } catch (const std::exception &e) {
            cerr << e.what() << endl;
            return -1;
        }
    }
    else { // sceneName != ""
        if (threadCount < 0) {
            threadCount = tbb::task_scheduler_init::automatic;
        }
        try {
            std::unique_ptr<NoriObject> root(loadFromXML(sceneName));
            /* When the XML root object is a scene, start rendering it .. */
            if (root->getClassType() == NoriObject::EScene)
                render(static_cast<Scene *>(root.get()), sceneName);
        } catch (const std::exception &e) {
            cerr << e.what() << endl;
            return -1;
        }
    }

    return 0;
}