/*****************************************************************************\
*                                                                           *
* File(s): HairModelGen.hpp                                            *
*                                                                           *
* Content: Example scene that shows minimum setup with an OpenGL capable   *
*          window, lighting setup, and a single moving object.              *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* FreeBSD License without any warranty or guaranty to work properly.        *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_HAIRMODELGEN_HPP__
#define __CFORGE_HAIRMODELGEN_HPP__
#define M_PI EIGEN_PI

#include "ExampleSceneBase.hpp"
/*
#include "pmp/algorithms/SurfaceGeodesic.cpp"
#include "pmp/SurfaceMesh.cpp"
#include "pmp/SurfaceMeshIO.cpp"
*/
#include "igl/dijkstra.h"
#include "igl/adjacency_list.h"
#include "igl/readMESH.h"
#include "igl/writeMESH.h"

using namespace Eigen;
using namespace std;

namespace CForge {

	class HairModelGen : public ExampleSceneBase {
	public:
		HairModelGen(void) {
			m_WindowTitle = "CrossForge Example - Minimum Graphics Setup";
			m_WinWidth = 1280;
			m_WinHeight = 720;
		}//Constructor

		~HairModelGen(void) {
			clear();
		}//Destructor

		bool compareVectorZ(Vector3f v1, Vector3f v2) {
			return (v1.z() < v2.z());
		}

		/*
		vector<Vector3f> getStartpoints(CForge::T3DMesh<float> *scalp, SurfaceMesh& surfaceScalp, Vector2f partingXY, int pointNumber = 10) {
			vector<Vector3f> startPoints;
			auto points = surfaceScalp.get_vertex_property<Point>("v:point");

			//get all Vertices with x ~ partingX and y >= partingY
			vector<Vertex> sampleVertices;
			
			float delta = 20.0f/scalp->vertexCount();		//tolerance around x
			for (auto i : surfaceScalp.vertices()) {
				Point p = points[i];
				if ((p[1] >= (partingXY.y() - delta)) && (p[1] <= (partingXY.y() + delta)))
					if ((p[0] >= (partingXY.x() - delta)) && (p[0] <= (partingXY.x() + delta))) sampleVertices.push_back(i);
			}
			
			//get vertex with highest and lowest z each
			//TODO Error when no or only 1 point(s) were sampled
			Vertex minZ, maxZ;

			if (sampleVertices.size() > 0) {
				minZ = sampleVertices[0];
				maxZ = sampleVertices[0];

				for (auto i : sampleVertices) {
					if (points[i][2] < points[minZ][2]) {
						minZ = i;
					} 
					else if (points[i][2] > points[maxZ][2]) {
						maxZ = i;
					}
				}
			}

			if (minZ != maxZ) {
				vector<Vertex> seeds;
				seeds.push_back(minZ); seeds.push_back(maxZ);

				//calculate geodesic distance for seeds
				vector<Vertex> neighbours;
				SurfaceGeodesic geod(surfaceScalp);
				unsigned int neighbourNum = geod.compute(seeds, std::numeric_limits<Scalar>::max(), INT_MAX, &neighbours);
				//geod.

				//convert to Vector3f
				vector<Vector3f> neighboursV;

				for (auto i : neighbours) {
					Point p = points[i];
					neighboursV.push_back(Vector3f(p[0], p[1], p[2]));
				}

				// extract pointNumber of points from neighboursV with approx. even distance
				int interval = neighbourNum / pointNumber;

					//we expect neighbours to be already sorted as of the nature of the geodesic algorithm
				for (int i = 0; i < neighbourNum; ++i) {
					//if ((i % interval) == 0) startPoints.push_back(neighboursV[i]);
					startPoints.push_back(neighboursV[i]);
				}

			}

			return startPoints;
		}
		*/

		vector<Vector3f> getStartpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f> &vertexList, vector<vector<int>> &adjacencyList, Vector2f partingXY, int pointNumber = 10) {
			vector<Vector3f> startPoints;
			//get all Vertices with x ~ partingX and y >= partingY
			vector<int> sampleVertices;

			float delta = 20.0f / scalp->vertexCount();		//tolerance around x
			for (int i = 0; i < vertexList.size(); i++) {
				if ((vertexList[i][1] >= (partingXY.y() - delta)) && (vertexList[i][1] <= (partingXY.y() + delta)))
					if ((vertexList[i][0] >= (partingXY.x() - delta)) && (vertexList[i][0] <= (partingXY.x() + delta))) sampleVertices.push_back(i);
			}

			//get vertex with highest and lowest z each
			//TODO Error when no or only 1 point(s) were sampled
			int minZ, maxZ;

			if (sampleVertices.size() > 0) {
				minZ = sampleVertices[0];
				maxZ = sampleVertices[0];

				for (auto i : sampleVertices) {
					if (vertexList[i][2] < vertexList[minZ][2]) {
						minZ = i;
					}
					else if (vertexList[i][2] > vertexList[maxZ][2]) {
						maxZ = i;
					}
				}
			}

			if (minZ != maxZ) {
				int num_v = scalp->vertexCount();
				int source = minZ;
				set<int> targets = { maxZ };
				VectorXd min_d(num_v);
				VectorXi previous(num_v);
				igl::dijkstra(source, targets, adjacencyList, min_d, previous);
				vector<int> path;
				igl::dijkstra(maxZ, previous, path);

				//TODO calculate geodesic distance

				//read positions Vector3f
				vector<Vector3f> pathV;

				for (auto i : path) {
					pathV.push_back(vertexList[i]);
				}

				// extract pointNumber of points from pathV with approx. even distance
				// only works for pathV.size() = x * pointNumber
				int interval = pathV.size() / pointNumber;

				//we expect path to be already sorted as of the nature of the geodesic algorithm
				for (int i = 0; i < pathV.size(); ++i) {
					//if ((i % interval) == 0) startPoints.push_back(neighboursV[i]);
					startPoints.push_back(pathV[i]);
				}

			}

			return startPoints;
		}

		void setSplineControlpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f> startPoints) {
			//TODO
			return;
		}

		void createSplines(CForge::T3DMesh<float>* scalp, vector<Vector3f> controlPoints) {
			//TODO
			return;
		}

		void createSideStrips() {
			//TODO
			return;
		}

		void createFrontStrips() {
			//TODO
			return;
		}

		void createBackStrips() {
			//TODO
			return;
		}

		void init() override{

			initWindowAndRenderDevice();
			initCameraAndLights();

			// load skydome and a scalp
			T3DMesh<float> M, Scalp;
			
			/*
			SAssetIO::load("Assets/ExampleScenes/SimpleSkydome.glb", &M);
			setMeshShader(&M, 0.8f, 0.04f);
			M.computePerVertexNormals();
			m_Skydome.init(&M);
			M.clear();
			*/
			/// gather textures for the skyboxes
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_right.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_left.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_up.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_down.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_back.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_front.png");

			m_Skybox.init(m_ClearSky[0], m_ClearSky[1], m_ClearSky[2], m_ClearSky[3], m_ClearSky[4], m_ClearSky[5]);

			SAssetIO::load("MyAssets/sphere.obj", &Scalp);
			setMeshShader(&Scalp, 0.1f, 0.04f);
			Scalp.computePerVertexNormals();
			m_Scalp.init(&Scalp);

			//testing
			//set Color for Mesh 
			for (uint32_t i = 0; i < Scalp.materialCount(); ++i) {
				T3DMesh<float>::Material* pMat = Scalp.getMaterial(i);
				pMat->Color = Vector4f(1.0f, 0.0f, 0.0f, 0.6f);
			}

			m_Test.init(&Scalp);

			// build scene graph
			m_RootSGN.init(nullptr);
			m_SG.init(&m_RootSGN);

			// add skybox
			// set initialize color adjustment values
			m_Skybox.brightness(1.15f);
			m_Skybox.contrast(1.1f);
			m_Skybox.saturation(1.2f);

			// create scene graph for the Skybox
			m_SkyboxTransSGN.init(nullptr);
			m_SkyboxGeomSGN.init(&m_SkyboxTransSGN, &m_Skybox);
			m_SkyboxSG.init(&m_SkyboxTransSGN);

			// add scalp
			m_HeadTransformSGN.init(&m_RootSGN, Vector3f(0.0f, 1.5f, 0.0f));
			m_ScalpTransformSGN.init(&m_HeadTransformSGN, Vector3f(0.0f, 0.0f, 0.0f));
			m_ScalpSGN.init(&m_ScalpTransformSGN, &m_Scalp);
			m_ScalpSGN.scale(Vector3f(1.0f, 1.0f, 1.0f));

			//testing
			vector <Vector3f> startPoints;
			Vector2f partingXY = Vector2f(0.1f, 0.1f);
			int stripNumber = 10;
			vector <Vector3f> vertexList;
			for (int i = 0; i < Scalp.vertexCount(); i++) {
				vertexList.push_back(Scalp.vertex(i));
			}
			vector<vector<int>> faceListIndex;
			for (int k = 0; k < Scalp.submeshCount(); k++) {
				vector<T3DMesh<float>::Face> faceList = Scalp.getSubmesh(k)->Faces;
				for (auto i : faceList) {
					vector<int> h;
					for (auto j : i.Vertices) {
						h.push_back(j);
					}
					faceListIndex.push_back(h);
				}
			}
			vector<vector<int>> adjList;
			igl::adjacency_list(faceListIndex, adjList);
			startPoints = getStartpoints(&Scalp, vertexList, adjList, partingXY, stripNumber);

			m_TestGroupSGN.init(&m_HeadTransformSGN);
			
			for (uint32_t i = 0; i < startPoints.size(); i++) {
				// create the scene graph nodes
				SGNGeometry* pGeomSGN = nullptr;
				SGNTransformation* pTransformSGN = nullptr;

				// initialize position and scaling
				pTransformSGN = new SGNTransformation();
				pTransformSGN->init(&m_TestGroupSGN);

				pTransformSGN->translation(startPoints[i]);
				//pTransformSGN->translation(Vector3f(0.0f, 0.0f, 0.0f));
				pTransformSGN->scale(Vector3f(0.04f, 0.04f, 0.04f));

				// initialize geometry
				pGeomSGN = new SGNGeometry();
				pGeomSGN->init(pTransformSGN, &m_Test);

				m_TestTransformSGNs.push_back(pTransformSGN);
				m_TestSGNs.push_back(pGeomSGN);

			}//for[stripNumber]

			// stuff for performance monitoring
			uint64_t LastFPSPrint = CForgeUtility::timestamp();
			int32_t FPSCount = 0;

			std::string ErrorMsg;
			if (0 != CForgeUtility::checkGLError(&ErrorMsg)) {
				SLogger::log("OpenGL Error" + ErrorMsg, "PrimitiveFactoryTestScene", SLogger::LOGTYPE_ERROR);
			}

		}//initialize

		void clear(void) override{
			m_RenderWin.stopListening(this);
			if (nullptr != m_pShaderMan) m_pShaderMan->release();
			m_pShaderMan = nullptr;
		}//clear


		void mainLoop(void)override {
			m_RenderWin.update();
			m_SG.update(60.0f / m_FPS);
			m_SkyboxSG.update(60.0f / m_FPS);

			defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());

			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera((VirtualCamera*)m_Sun.camera());
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			m_RenderDev.activeCamera(&m_Cam);
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
			// Skybox should be last thing to render
			m_SkyboxSG.render(&m_RenderDev);

			m_RenderWin.swapBuffers();

			updateFPS();

			defaultKeyboardUpdate(m_RenderWin.keyboard());

			std::string ErrorMsg;
			if (0 != CForgeUtility::checkGLError(&ErrorMsg)) {
				SLogger::log("OpenGL Error" + ErrorMsg, "PrimitiveFactoryTestScene", SLogger::LOGTYPE_ERROR);
			}
		}



	protected:

		// Scene Graph
		SGNTransformation m_RootSGN;
		SceneGraph m_SkyboxSG;
		SGNTransformation m_SkyboxTransSGN;
		SGNGeometry m_SkyboxGeomSGN;
		SGNGeometry m_ScalpSGN;
		SGNTransformation m_ScalpTransformSGN;
		SGNTransformation m_HeadTransformSGN;

		vector<string> m_ClearSky;
		SkyboxActor m_Skybox;
		StaticActor m_Scalp;

		StaticActor m_Test;
		std::vector<SGNTransformation*> m_TestTransformSGNs;
		std::vector<SGNGeometry*> m_TestSGNs;

		SGNTransformation m_TestGroupSGN;

	};//HairModelGen

}//name space

#endif