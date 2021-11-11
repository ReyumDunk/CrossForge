#ifndef TERRAIN_SETUP_HPP
#define TERRAIN_SETUP_HPP

#include "CForge/Graphics/Shader/SShaderManager.h"
#include "CForge/Graphics/GLWindow.h"
#include "CForge/Graphics/GraphicsUtility.h"
#include "CForge/Graphics/RenderDevice.h"
#include "CForge/Graphics/Lights/DirectionalLight.h"
#include "CForge/Graphics/SceneGraph/SceneGraph.h"
#include "CForge/Graphics/SceneGraph/SGNGeometry.h"
#include "CForge/Graphics/SceneGraph/SGNTransformation.h"
#include <glad/glad.h>
#include <CForge/Graphics/STextureManager.h>

#include "TileNode.h"

using namespace CForge;
using namespace Eigen;
using namespace std;

namespace Terrain {

    void initCForge(GLWindow* window, RenderDevice* renderDevice, VirtualCamera* camera, DirectionalLight* sun) {
        uint32_t winWidth = 720;
        uint32_t winHeight = 720;

        window->init(Vector2i(100, 100), Vector2i(winWidth, winHeight), "Terrain Setup");
        gladLoadGL();

        string GLError;
        GraphicsUtility::checkGLError(&GLError);
        if (!GLError.empty()) printf("GLError occurred: %s\n", GLError.c_str());

        RenderDevice::RenderDeviceConfig renderConfig;
        renderConfig.DirectionalLightsCount = 1;
        renderConfig.PointLightsCount = 0;
        renderConfig.SpotLightsCount = 0;
        renderConfig.ExecuteLightingPass = true;
        renderConfig.GBufferWidth = winWidth;
        renderConfig.GBufferHeight = winHeight;
        renderConfig.pAttachedWindow = window;
        renderConfig.PhysicallyBasedShading = true;
        renderConfig.UseGBuffer = true;
        renderDevice->init(&renderConfig);

        ShaderCode::LightConfig lightConfig;
        lightConfig.DirLightCount = 1;
        lightConfig.PCFSize = 1;
        lightConfig.PointLightCount = 0;
        lightConfig.ShadowBias = 0.0004f;
        lightConfig.ShadowMapCount = 1;
        lightConfig.SpotLightCount = 0;
        SShaderManager* shaderManager = SShaderManager::instance();
        shaderManager->configShader(lightConfig);
        shaderManager->release();

        camera->init(Vector3f(.0f, 500.0f, 100.0f), Vector3f::UnitY());
        camera->pitch(GraphicsUtility::degToRad(-75.0f));
        camera->projectionMatrix(winWidth, winHeight, GraphicsUtility::degToRad(45.0f), 0.1f, 10000.0f);
        renderDevice->activeCamera(camera);

        Vector3f sunPos = Vector3f(10.0f, 100.0f, 0.0f);
        sun->init(sunPos, -sunPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 2.0f);
        renderDevice->addLight(sun);
    }

    void updateCamera(Mouse* mouse, Keyboard* keyboard, VirtualCamera* camera) {
        if (nullptr == keyboard) return;

        const float movementSpeed = 2;

        float MovementScale = 1.0f;
        if (keyboard->keyPressed(Keyboard::KEY_LEFT_SHIFT) || keyboard->keyPressed(Keyboard::KEY_RIGHT_SHIFT)) {
            MovementScale = 4.0f;
        }

        if (keyboard->keyPressed(Keyboard::KEY_W) || keyboard->keyPressed(Keyboard::KEY_UP)) camera->forward(movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_S) || keyboard->keyPressed(Keyboard::KEY_DOWN)) camera->forward(-movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_A) || keyboard->keyPressed(Keyboard::KEY_LEFT)) camera->right(-movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_D) || keyboard->keyPressed(Keyboard::KEY_RIGHT)) camera->right(movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_LEFT_ALT)) camera->up(-movementSpeed * MovementScale);
        if (keyboard->keyPressed(Keyboard::KEY_LEFT_CONTROL)) camera->up(movementSpeed * MovementScale);

        // rotation
        if (mouse->buttonState(Mouse::BTN_RIGHT)) {
            Vector2f MouseDelta = mouse->movement();

            camera->rotY(GraphicsUtility::degToRad(MouseDelta.x()) * -0.1f * movementSpeed);
            camera->pitch(GraphicsUtility::degToRad(MouseDelta.y()) * -0.1f * movementSpeed);

            mouse->movement(Eigen::Vector2f::Zero());
        }
    }

    void spawnClipmapTiles(SGNTransformation* mapTransform, Tile& tile, vector<TileNode*>& tileNodes) {
        const uint32_t LOD_LEVELS = 6; // Todo: move into tile
        const tuple<Tile::TileVariant, int> TILE_ALIGNMENTS[4][4] = {
            {
                {Tile::Corner, 0},
                {Tile::Edge, 3},
                {Tile::Edge, 3},
                {Tile::Corner, 3},
            },
            {
                {Tile::Edge, 0},
                {Tile::Normal, 0},
                {Tile::Normal, 0},
                {Tile::Edge, 2},
            },
            {
                {Tile::Edge, 0},
                {Tile::Normal, 0},
                {Tile::Normal, 0},
                {Tile::Edge, 2},
            },
            {
                {Tile::Corner, 1},
                {Tile::Edge, 1},
                {Tile::Edge, 1},
                {Tile::Corner, 2},
            },
        };

        float sideLength = static_cast<float>(tile.sideLength());
        float lineOffset = sideLength * 1.5f;
        Vector2f linePositions[4] = {
            Vector2f(-lineOffset, 1.0f),
            Vector2f(1.0f, lineOffset + 2.0f),
            Vector2f(lineOffset + 2.0f, 1.0f),
            Vector2f(1.0f, -lineOffset),
        };

        TileNode::TileData data = {Vector2f(2, 2), 0, 2, Tile::Cross};
        tileNodes.push_back(new TileNode(mapTransform, &tile, data));

        for (int level = 0; level < LOD_LEVELS; level++) {
            int lod = 2 << level;
            float scale = static_cast<float>(lod);

            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    if (level == 0 || x == 0 || x == 3 || y == 0 || y == 3) {
                        auto[variant, orientation] = TILE_ALIGNMENTS[y][x];

                        auto pos = Vector2f((static_cast<float>(x) - 1.5f) * sideLength * scale,
                                            (static_cast<float>(y) - 1.5f) * sideLength * scale) +
                                   Vector2f(x < 2 ? 0 : 2, y < 2 ? 0 : 2) * lod;

                        data = {pos, orientation, lod, variant};
                        tileNodes.push_back(new TileNode(mapTransform, &tile, data));
                    }
                }

                data = {linePositions[y] * scale, y, lod, Tile::Line};
                tileNodes.push_back(new TileNode(mapTransform, &tile, data));
            }

            data = {Vector2f(scale, scale), 0, lod, Tile::Trim};
            tileNodes.push_back(new TileNode(mapTransform, &tile, data));
        }
    }

	void TerrainSetup() {
        bool wireframe = false;

        GLWindow window;
        RenderDevice renderDevice;
        VirtualCamera camera;
        DirectionalLight sun;
        initCForge(&window, &renderDevice, &camera, &sun);

        vector<TileNode*> tileNodes;

        auto texture = STextureManager::create("Assets/height_map1.jpg");

        Tile tile = Tile(64, texture);

        SGNTransformation rootTransform;
        rootTransform.init(nullptr);
        spawnClipmapTiles(&rootTransform, tile, tileNodes);
        SceneGraph sceneGraph;
        sceneGraph.init(&rootTransform);

		while (!window.shutdown()) {
			window.update();

			sceneGraph.update(1.0f);

            for (auto node : tileNodes) {
                node->update(camera.position().x(), camera.position().z());
            }

			renderDevice.activePass(RenderDevice::RENDERPASS_GEOMETRY);

            if (wireframe) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(2);
			    sceneGraph.render(&renderDevice);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
            else {
                sceneGraph.render(&renderDevice);
            }

			renderDevice.activePass(RenderDevice::RENDERPASS_LIGHTING);

			window.swapBuffers();

            updateCamera(window.mouse(), window.keyboard(), renderDevice.activeCamera());

            if (window.keyboard()->keyPressed(Keyboard::KEY_ESCAPE)) {
                window.closeWindow();
            }
            if (window.keyboard()->keyPressed(Keyboard::KEY_F1)) {
                window.keyboard()->keyState(Keyboard::KEY_F1, Keyboard::KEY_RELEASED);
                wireframe = !wireframe;
            }
		}

        for (auto node : tileNodes) {
            delete node;
        }
	}
}

#endif