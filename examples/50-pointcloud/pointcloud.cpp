/*
 * Copyright 2011-2017 Branimir Karadzic. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx#license-bsd-2-clause
 */

#include "../../src/config.h"
#include "common.h"
#include "bgfx_utils.h"
#include "camera.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include <vector>
#include <cstdlib>
#include <ctime>

#include <thread>
#include <mutex>
#include <future>
#include <chrono>

const float CAMERA_FOV_X = 45.0f;
const float CAMERA_FOV_Y = 45.0f;

#pragma pack(push, 1) 
struct PosColorVertex
{
	float m_x;
	float m_y;
	float m_z;
	uint32_t m_abgr;

	static void init()
	{
		ms_decl
			.begin()
			.add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
			.add(bgfx::Attrib::Color0,   4, bgfx::AttribType::Uint8, true)
			.end();
	};

	static bgfx::VertexDecl ms_decl;
};
#pragma pack(pop)

bgfx::VertexDecl PosColorVertex::ms_decl;

void g_PointCloudUpdated(void* _ptr, void* _userData);

const int POINT_COUNT = BGFX_CONFIG_DYNAMIC_VERTEX_BUFFER_SIZE / 16;
const bool DYNAMIC_POINT_CLOUD = true;

class ExamplePointCloud : public entry::AppI
{
private:
	friend void g_PointCloudUpdated(void* _ptr, void* _userData);

	std::mutex m_NewDataLock;
	std::shared_ptr<std::vector<PosColorVertex>> m_NewData;
	bool m_NewDataUploading;

	// Background thread that continuously pushes new point cloud data.
	std::thread m_BackgroundThread;
	std::shared_ptr<std::promise<void>> m_BackgroundThreadShutdown;

	/// Generate a new point cloud.
    std::shared_ptr<std::vector<PosColorVertex>> GeneratePointCloud()
    {
        std::srand(static_cast<unsigned int>(std::time(0)));

		auto pointCloud = std::make_shared<std::vector<PosColorVertex>>();
        pointCloud->reserve(POINT_COUNT);

        for (int i = 0; i < POINT_COUNT; i++)
        {
            const float CUBE_SIZE = 100;

			unsigned color = (0xff000000 |
				((std::rand() & 0xff) << 16) |
				((std::rand() & 0xff) << 8) |
				((std::rand() & 0xff) << 0));
			color = std::rand();

            PosColorVertex v =
            {
				static_cast<float>((double)std::rand() / RAND_MAX) * CUBE_SIZE - CUBE_SIZE / 2,
				static_cast<float>((double)std::rand() / RAND_MAX) * CUBE_SIZE - CUBE_SIZE / 2,
				static_cast<float>((double)std::rand() / RAND_MAX) * CUBE_SIZE - CUBE_SIZE / 2,
                color
            };
			pointCloud->push_back(v);
        }

		return pointCloud;
    }

	void init(int _argc, char** _argv) BX_OVERRIDE
	{
		m_width  = 1280;
		m_height = 720;
		m_debug  = BGFX_DEBUG_TEXT;
		m_reset  = BGFX_RESET_VSYNC;
        
		auto renderType = bgfx::RendererType::Count;
#if defined(_WIN32)
		renderType = bgfx::RendererType::OpenGL;
#endif

		bgfx::init(renderType);
		bgfx::reset(m_width, m_height, m_reset);

		// Enable debug text.
		bgfx::setDebug(m_debug);

		// Set view 0 clear state.
		bgfx::setViewClear(0
				, BGFX_CLEAR_COLOR|BGFX_CLEAR_DEPTH
				, 0x303030ff
				, 1.0f
				, 0
				);

		// Create vertex stream declaration.
		PosColorVertex::init();

		m_NewDataUploading = false;
		m_NewData = GeneratePointCloud();
		m_CurrentPointCloud = bgfx::createDynamicVertexBuffer(bgfx::makeRef((*m_NewData).data(), POINT_COUNT * 16),
			PosColorVertex::ms_decl);
		m_UploadPointCloud.idx = 0;
		if (DYNAMIC_POINT_CLOUD)
		{
			m_UploadPointCloud = bgfx::createDynamicVertexBuffer(POINT_COUNT, PosColorVertex::ms_decl);
		}

		// Create program from shaders.
        m_pointSize = bgfx::createUniform("u_pointSize", bgfx::UniformType::Vec4);
		m_program = loadProgram("vs_pointcloud", "fs_pointcloud");

        cameraCreate();

        const float initialPos[3] = { 15.5f, 0.0f, -15.5f };
        cameraSetPosition(initialPos);
        cameraSetHorizontalAngle(bx::toRad(-CAMERA_FOV_X / 2));

		m_timeOffset = bx::getHPCounter();

		if (DYNAMIC_POINT_CLOUD)
		{
			m_BackgroundThreadShutdown = std::make_shared<std::promise<void>>();
			m_BackgroundThread = std::thread([this]()
			{
				auto shutdownFuture = m_BackgroundThreadShutdown->get_future();

				while (true)
				{
					{
						std::lock_guard<std::mutex> lock(m_NewDataLock);

						// generate new data if current has not yet been uploaded
						if (!m_NewData)
						{
							m_NewData = GeneratePointCloud();
						}
					}

					if (shutdownFuture.wait_for(std::chrono::seconds(1)) ==
						std::future_status::ready)
					{
						break;
					}
				}
			});
		}
	}

	virtual int shutdown() BX_OVERRIDE
	{
		if (DYNAMIC_POINT_CLOUD)
		{
			try
			{
				if (m_BackgroundThread.joinable())
				{
					m_BackgroundThreadShutdown->set_value();

					m_BackgroundThread.join();
				}
			}
			catch (...)
			{
			}
		}

        // Cleanup.
        cameraDestroy();

		if (isValid(m_CurrentPointCloud))
		{
			bgfx::destroyDynamicVertexBuffer(m_CurrentPointCloud);
		}

		if (isValid(m_UploadPointCloud))
		{
			bgfx::destroyDynamicVertexBuffer(m_UploadPointCloud);
		}
		
        bgfx::destroyUniform(m_pointSize);
		bgfx::destroyProgram(m_program);

		// Shutdown bgfx.
		bgfx::shutdown();

		return 0;
	}

	bool update() BX_OVERRIDE
	{
        if (!entry::processWindowEvents(m_state, m_debug, m_reset) )
		{
			int64_t now = bx::getHPCounter();
			static int64_t last = now;
			const int64_t frameTime = now - last;
			last = now;
			const double freq = double(bx::getHPFrequency() );
            const double toMs = 1000.0 / freq;
            const float time = (float)((now - m_timeOffset) / double(bx::getHPFrequency()));
            const float deltaTime = float(frameTime / freq);

			// Use debug font to print information about this example.
			bgfx::dbgTextClear();
			bgfx::dbgTextPrintf(0, 1, 0x4f, "bgfx/examples/50-pointcloud");
			bgfx::dbgTextPrintf(0, 2, 0x6f, "Description: Rendering a point-cloud.");
			bgfx::dbgTextPrintf(0, 3, 0x0f, "Frame: % 7.3f[ms]", double(frameTime)*toMs);

            uint32_t width = m_state.m_width;
            uint32_t height = m_state.m_height;

            // Update camera.
            float view[16];
            cameraUpdate(deltaTime, m_state.m_mouse);
            cameraGetViewMtx(view);

            float heightNearPlane = 1.0f;

			// Set view and projection matrix for view 0.
            {
                float proj[16];
                bx::mtxProj(proj, CAMERA_FOV_Y, float(width) / float(height), 0.1f, 10000.0f, 
                    bgfx::getCaps()->homogeneousDepth);

                bgfx::setViewTransform(0, view, proj);
                bgfx::setViewRect(0, 0, 0, uint16_t(width), uint16_t(height));

                bgfx::setViewTransform(1, view, proj);
                bgfx::setViewRect(1, 0, 0, uint16_t(width), uint16_t(height));

                float at[3] = { 0.0f, 0.0f, 0.0f };
                float eye[3] = { 17.5f, 10.0f, -17.5f };
                bx::mtxLookAt(view, eye, at);

                bgfx::setViewTransform(2, view, proj);
                bgfx::setViewRect(2, 10, uint16_t(height - height / 4 - 10), uint16_t(width / 4), uint16_t(height / 4));

                const float p11 = proj[5];
                // const float p11_calc = static_cast<float>(1.0 / std::tan(0.5 * CAMERA_FOV_Y * M_PI / 180));
                const float viewPortHeight = 1; // fullscreen

				// http://stackoverflow.com/questions/25780145/gl-pointsize-corresponding-to-world-space-size
                heightNearPlane = viewPortHeight * p11;
            }

			// This dummy draw call is here to make sure that view 0 is cleared
			// if no other draw calls are submitted to view 0.
            bgfx::touch(0);

			if (DYNAMIC_POINT_CLOUD)
            {
                std::lock_guard<std::mutex> lock(m_NewDataLock);
                if (m_NewData && !m_NewDataUploading)
                {
					m_NewDataUploading = true;
					bgfx::updateDynamicVertexBuffer(m_UploadPointCloud, // handle
						0, // offset
						bgfx::makeRef((*m_NewData).data(), POINT_COUNT * 16, &g_PointCloudUpdated, this));
                }
            }

            // render point cloud
            {
                float mtx[16];
                bx::mtxIdentity(mtx);

                bgfx::setTransform(mtx);

                float pointSize[4];
                pointSize[0] = heightNearPlane;
                pointSize[1] = 200;
                pointSize[2] = 0;
                pointSize[3] = 0;

                bgfx::setUniform(m_pointSize, pointSize);

                // Set vertex and index buffer.
                bgfx::setVertexBuffer(0, m_CurrentPointCloud);

                // Set render states.
                bgfx::setState(0
                    | BGFX_STATE_DEFAULT
                    | BGFX_STATE_PT_POINTS
                    );

                // Submit primitive for rendering to view 0.
                bgfx::submit(0, m_program);
            }

			// Advance to next frame. Rendering thread will be kicked to
			// process submitted rendering primitives.
			bgfx::frame();

			return true;
		}

		return false;
	}

	uint32_t m_width;
	uint32_t m_height;
	uint32_t m_debug;
	uint32_t m_reset;

    bgfx::DynamicVertexBufferHandle m_CurrentPointCloud;
    bgfx::DynamicVertexBufferHandle m_UploadPointCloud;

	bgfx::ProgramHandle m_program;
    bgfx::UniformHandle m_pointSize;

	int64_t m_timeOffset;

    entry::WindowState m_state;
};

void g_PointCloudUpdated(void* _ptr, void* _userData)
{
	ExamplePointCloud* example = reinterpret_cast<ExamplePointCloud*>(_userData);

	std::lock_guard<std::mutex> lock(example->m_NewDataLock);

	std::swap(example->m_CurrentPointCloud, example->m_UploadPointCloud);

	example->m_NewData.reset();
	example->m_NewDataUploading = false;
}

ENTRY_IMPLEMENT_MAIN(ExamplePointCloud);
