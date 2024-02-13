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
#include "tinynurbs/tinynurbs.h"
#include "glm/glm.hpp"

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

		static bool compareVectorZ(const int &v1, const int &v2) {
			return (vertexList[v1].z() < vertexList[v2].z());
		}

		glm::vec3 glmVecFromVector3f(Vector3f vector) {
			return glm::vec3(vector.x(), vector.y(), vector.z());
		}

		Vector3f Vector3fFromGlmVec3(glm::vec3 vector) {
			return Vector3f(vector.x, vector.y, vector.z);
		}

		/* pmp usage
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

		vector<int> getStartpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f> &vertexList, vector<vector<int>> &adjacencyList, Vector2f partingXY, int pointNumber = 10) {
			//TODO change Vector3f vertex representation to index rep.
			vector<Vector3f> startPoints;
			//get all Vertices with x ~ partingX and y >= partingY
			vector<int> sampleVertices;

			float delta = 30.0f / scalp->vertexCount();		//tolerance around x
			for (int i = 0; i < vertexList.size(); i++) {
				if (vertexList[i][1] >= (partingXY.y() - delta))
					if ((vertexList[i][0] >= (partingXY.x() - delta)) && (vertexList[i][0] <= (partingXY.x() + delta))) sampleVertices.push_back(i);
			}

			//get vertex with highest and lowest z each
			//TODO Error when no or only 1 point(s) were sampled
			int minZ, maxZ{};
			sort(sampleVertices.begin(), sampleVertices.end(), CForge::HairModelGen::compareVectorZ);
			if (sampleVertices.size() > 0) {
				minZ = sampleVertices[0];
				maxZ = sampleVertices[sampleVertices.size() - 1];
			}
			/*
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
			}*/

			vector<int> path;
			if (minZ != maxZ) {
				printf("%d %d\n", minZ, maxZ);
				int num_v = scalp->vertexCount();
				int source = minZ;
				set<int> targets = { maxZ };
				VectorXd min_d(num_v);
				VectorXi previous(num_v);
				igl::dijkstra(source, targets, adjacencyList, min_d, previous);
				//printf("previous: %d\n", previous.size());
				igl::dijkstra(maxZ, previous, path);

				//TODO calculate geodesic distance
				
				//read positions Vector3f
				printf("Pathindices:\n");
				for (auto i : path){
					printf("%d", i);
				}
				/*
				vector<Vector3f> pathV;

				for (auto i : path) {
					pathV.push_back(vertexList[i]);
				}

				// extract pointNumber of points from pathV with approx. even distance
				// only works for pathV.size() = x * pointNumber
				int interval = pathV.size() / pointNumber;

				//TODO if not sorted
				for (int i = 0; i < pathV.size(); ++i) {
					//if ((i % interval) == 0) startPoints.push_back(neighboursV[i]);
					startPoints.push_back(pathV[i]);
				}
				*/
			}

			return path;
		}
		vector<int> getTESTStartpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f>& vertexList, Vector2f partingXY, int pointNumber = 10) {
			vector<int> startPoints;
			//get all Vertices with x ~ partingX and y >= partingY
			vector<int> sampleVertices;

			float delta = 30.0f / scalp->vertexCount();		//tolerance around x
			for (int i = 0; i < vertexList.size(); i++) {
				if (vertexList[i][1] >= (partingXY.y() - delta))
					if ((vertexList[i][0] >= (partingXY.x() - delta)) && (vertexList[i][0] <= (partingXY.x() + delta))) sampleVertices.push_back(i);
			}

			for (auto i : sampleVertices) {
				startPoints.push_back(i);
			}
			sort(startPoints.begin(), startPoints.end(), CForge::HairModelGen::compareVectorZ);
			startPoints.erase(startPoints.begin() + 9, startPoints.end() - 9);
			//problem: multiple points in (nearly?) same position
			/*
			int j = 0;
			while (j < 5) {
				startPoints.erase(startPoints.begin() + j + 1, startPoints.begin() + j + 3);
				j++;
			}*/
			/*
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin());
			startPoints.erase(startPoints.begin() + 1);*/
			return startPoints;
		}
		
		vector<vector<Vector3f>> setSplineControlpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f>& vertexList, vector<Vector3f>& normalList, vector<int> startPoints, float minY) {
			vector<vector<Vector3f>> controlPoints;
			T3DMesh<float>::AABB scalpAABB = scalp->aabb();
			float ctrlPointNum = 6.0f;
			Vector3f middleStartPoint = vertexList[startPoints[(int)startPoints.size() / 2]];
			//approximate max length is double diagonal length between start and end point divided by sqrt(2) (3 controlpoints)
			//pythagoras on isosceles triangle: sqrt(2)*a = c -> 2*a = 2*c/sqrt(2)
			//additionally divided by number of desired controlpoints-1
			float vecLength = (Vector3f(scalpAABB.Max.x(), minY, 0.0f) - middleStartPoint).stableNorm();
			vecLength = 2.0f / 1.4f * vecLength / (ctrlPointNum - 1.1f);		//smaller value for controlPoint number so that it guarantees constant number in creation loop

			//left side
			float initialAngleLeft = -80.0f;
			
			for (int i = 0; i < startPoints.size(); i++) {
				Vector3f currentPoint = vertexList[startPoints[i]];
				vector<Vector3f> currentCtrlPts;
				currentCtrlPts.push_back(currentPoint);
				//get normal from startPoints
				Vector3f currentVec = normalList[startPoints[i]];
				//get initial rotation vector
				Vector3f currentRotVec;
				if (i == 0) {
					currentRotVec = vertexList[startPoints[i + 1]] - currentPoint;
				}
				else if (i == startPoints.size() - 1) {
					currentRotVec = currentPoint - vertexList[startPoints[i - 1]];
				}
				else {
					currentRotVec = currentPoint - vertexList[startPoints[i - 1]] + vertexList[startPoints[i + 1]] - currentPoint;
				}
				//currentRotVec = Vector3f(0.0f, currentRotVec.y(), currentRotVec.z());
				currentRotVec.normalize();
				//apply initial rotation to currentVec 
				currentVec.normalize();
				Vector4f h;
				h = Vector4f(currentVec.x(), currentVec.y(), currentVec.z(), 1.0f);
				
				h = CForgeMath::rotationMatrix((Quaternionf)AngleAxisf(CForgeMath::degToRad(initialAngleLeft), currentRotVec)) * h;
				currentVec = Vector3f(h.x(), h.y(), h.z());
				//apply length to currentVec -> about 4(-6?) controlpoints
				currentVec.normalize();
				currentVec = vecLength * currentVec;
				
				//control point creation loop per Spline
				for (float i = 1.0f; i < ctrlPointNum - 1.0f; i+=1.0f)  {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					//rotate currentVec, rotation angle depends on horizontal position of parting on scalp
					float rotAngle = (-180.0f - initialAngleLeft) / (ctrlPointNum - 2.0f) + middleStartPoint.x()*(-100.0f);		//TODO adjust angle added according to position
					Vector4f h;
					h = Vector4f(currentVec.x(), currentVec.y(), currentVec.z(), 1.0f);
					if (h.x() > 0.0f) {
						h = CForgeMath::rotationMatrix((Quaternionf)AngleAxisf(CForgeMath::degToRad(rotAngle), currentRotVec)) * h;
					}
					//prevent resulting vector from pointing in opposite x direction from before
					if (h.x() < 0.0f) currentVec = Vector3f(0.0f, h.y(), h.z());
					else currentVec = Vector3f(h.x(), h.y(), h.z());
					//apply gravity and then length
					currentVec += Vector3f(0.0f, -0.05f, 0.0f);		//TODO adjust gravity value
					currentVec.normalize();
					currentVec = vecLength * currentVec;			
					//TODO IMPROVEMENT make vecLength proportional to number of current ctrl point so that earlier length is shorter
					
				}
				//last controlpoint is at minY
				Vector3f lastPoint = Vector3f(currentPoint.x(), minY, currentPoint.z());
				currentCtrlPts.push_back(lastPoint);
				controlPoints.push_back(currentCtrlPts);
				currentCtrlPts.clear();
			}

			//right side same as left side only diffrent rotation angles and negative x TODO
			float initialAngleRight = 80.0f;
			
			for (int i = 0; i < startPoints.size(); i++) {
				Vector3f currentPoint = vertexList[startPoints[i]];
				vector<Vector3f> currentCtrlPts;
				currentCtrlPts.push_back(currentPoint);
				//get normal from startPoints
				Vector3f currentVec = normalList[startPoints[i]];
				//get initial rotation vector
				Vector3f currentRotVec;
				if (i == 0) {
					currentRotVec = vertexList[startPoints[i + 1]] - currentPoint;
				}
				else if (i == startPoints.size() - 1) {
					currentRotVec = currentPoint - vertexList[startPoints[i - 1]];
				}
				else {
					currentRotVec = currentPoint - vertexList[startPoints[i - 1]] + vertexList[startPoints[i + 1]] - currentPoint;
				}
				//currentRotVec = Vector3f(0.0f, currentRotVec.y(), currentRotVec.z());
				currentRotVec.normalize();
				//apply initial rotation to currentVec 
				currentVec.normalize();
				Vector4f h;
				h = Vector4f(currentVec.x(), currentVec.y(), currentVec.z(), 1.0f);

				h = CForgeMath::rotationMatrix((Quaternionf)AngleAxisf(CForgeMath::degToRad(initialAngleRight), currentRotVec)) * h;
				currentVec = Vector3f(h.x(), h.y(), h.z());
				//apply length to currentVec -> about 4(-6?) controlpoints
				currentVec.normalize();
				currentVec = vecLength * currentVec;

				//control point creation loop per Spline
				for (float i = 1.0f; i < ctrlPointNum - 1.0f; i += 1.0f) {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					//rotate currentVec, rotation angle depends on horizontal position of parting on scalp
					float rotAngle = (180.0f - initialAngleRight) / (ctrlPointNum - 2.0f) + middleStartPoint.x() * (-100.0f);		//TODO adjust angle added according to position
					Vector4f h;
					h = Vector4f(currentVec.x(), currentVec.y(), currentVec.z(), 1.0f);
					if (h.x() < 0.0f) {
						h = CForgeMath::rotationMatrix((Quaternionf)AngleAxisf(CForgeMath::degToRad(rotAngle), currentRotVec)) * h;
					}
					//prevent resulting vector from pointing in opposite x direction from before
					if (h.x() > 0.0f) currentVec = Vector3f(0.0f, h.y(), h.z());
					else currentVec = Vector3f(h.x(), h.y(), h.z());
					//apply gravity and then length
					currentVec += Vector3f(0.0f, -0.05f, 0.0f);		//TODO adjust gravity value
					currentVec.normalize();
					currentVec = vecLength * currentVec;
					//TODO IMPROVEMENT make vecLength proportional to number of current ctrl point so that earlier length is shorter

				}
				//last controlpoint is at minY
				Vector3f lastPoint = Vector3f(currentPoint.x(), minY, currentPoint.z());
				currentCtrlPts.push_back(lastPoint);
				controlPoints.push_back(currentCtrlPts);
				currentCtrlPts.clear();
			}
			return controlPoints;
		}
		
		vector<tinynurbs::Curve<float>> createSplines(CForge::T3DMesh<float>* scalp, vector<vector<Vector3f>> controlPoints) {
			vector<tinynurbs::Curve<float>> splineList;
			for (auto i : controlPoints) {
				tinynurbs::Curve<float> spline;
				vector<glm::vec3> glmCtrlPts;
				for (auto j : i) {
					glmCtrlPts.push_back(glmVecFromVector3f(j));
				}
				spline.control_points = glmCtrlPts;
				// #knots == #control points + degree + 1
				vector<float> knots;
				for (int i = 0; i < glmCtrlPts.size(); i++) knots.push_back(0.0f);
				spline.degree = 3;
				for (int i = 0; i < spline.degree + 1; i++) knots.push_back(1.0f);
				spline.knots = knots;
				if (!tinynurbs::curveIsValid(spline)) {
					printf("Spline not valid\n");
				}
				splineList.push_back(spline);
			}
			return splineList;
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

		void TestStartPoints(SGNTransformation* headTransformSGN, vector<int> startPoints, vector<Vector3f> vertexList, StaticActor* testActor) {
			
			m_TestStartPointsGroupSGN.init(headTransformSGN);

			for (uint32_t i = 0; i < startPoints.size(); i++) {
				// create the scene graph nodes
				SGNGeometry* pGeomSGN = nullptr;
				SGNTransformation* pTransformSGN = nullptr;

				// initialize position and scaling
				pTransformSGN = new SGNTransformation();
				pTransformSGN->init(&m_TestStartPointsGroupSGN);

				pTransformSGN->translation(vertexList[startPoints[i]]);
				//pTransformSGN->translation(Vector3f(0.0f, 0.0f, 0.0f));
				pTransformSGN->scale(Vector3f(0.04f, 0.04f, 0.04f));

				// initialize geometry
				pGeomSGN = new SGNGeometry();
				pGeomSGN->init(pTransformSGN, testActor);

				m_TestStartPointsTransformSGNs.push_back(pTransformSGN);
				m_TestStartPointsSGNs.push_back(pGeomSGN);

			}//for[stripNumber]
		}

		void TestControlPoints(SGNTransformation* headTransformSGN, vector<vector<Vector3f>> controlPoints, StaticActor* testActor) {

			m_TestControlPointsGroupSGN.init(headTransformSGN);

			for (auto i : controlPoints) {
				for (auto j : i) {
					// create the scene graph nodes
					SGNGeometry* pGeomSGN = nullptr;
					SGNTransformation* pTransformSGN = nullptr;

					// initialize position and scaling
					pTransformSGN = new SGNTransformation();
					pTransformSGN->init(&m_TestControlPointsGroupSGN);

					pTransformSGN->translation(j);
					//pTransformSGN->translation(Vector3f(0.0f, 0.0f, 0.0f));
					pTransformSGN->scale(Vector3f(0.04f, 0.04f, 0.04f));

					// initialize geometry
					pGeomSGN = new SGNGeometry();
					pGeomSGN->init(pTransformSGN, testActor);

					m_TestControlPointsTransformSGNs.push_back(pTransformSGN);
					m_TestControlPointsSGNs.push_back(pGeomSGN);
				}//for[controlPointsNumber]
			}//for[stripNumber]
		}

		void TestSplines(SGNTransformation* headTransformSGN, vector<tinynurbs::Curve<float>> splineList, StaticActor* testActor) {
			m_TestSplinesGroupSGN.init(headTransformSGN);
			for (auto i : splineList) {
				for (float j = 0.0f; j <= 1.0f; j+=1.0f/sampleRate) {
					// create the scene graph nodes
					SGNGeometry* pGeomSGN = nullptr;
					SGNTransformation* pTransformSGN = nullptr;

					// initialize position and scaling
					pTransformSGN = new SGNTransformation();
					pTransformSGN->init(&m_TestSplinesGroupSGN);

					pTransformSGN->translation(Vector3fFromGlmVec3(tinynurbs::curvePoint(i, j)));
					//pTransformSGN->translation(Vector3f(0.0f, 0.0f, 0.0f));
					pTransformSGN->scale(Vector3f(0.01f, 0.01f, 0.01f));

					// initialize geometry
					pGeomSGN = new SGNGeometry();
					pGeomSGN->init(pTransformSGN, testActor);

					m_TestSplinesTransformSGNs.push_back(pTransformSGN);
					m_TestSplinesSGNs.push_back(pGeomSGN);
				}//for[controlPointsNumber]
			}//for[stripNumber]
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
			setMeshShader(&Scalp, 0.5f, 0.04f);
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
			vector <int> startPoints;
			Vector2f partingXY = Vector2f(0.1f, 0.3f);
			int stripNumber = 10;
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
			//printf("%d, %d \n", Scalp.vertexCount(), adjList.size());
			//printf("%d, %d \n", Scalp.vertexCount(), Scalp.normalCount());
			//startPoints = getStartpoints(&Scalp, vertexList, adjList, partingXY, stripNumber);
			startPoints = getTESTStartpoints(&Scalp, vertexList, partingXY, stripNumber);
			//printf("%d \n", startPoints.size());
			vector <Vector3f> normalList;
			for (int i = 0; i < Scalp.normalCount(); i++) {
				normalList.push_back(Scalp.normal(i));
			}
			//stop height for hair
			float minY = -0.5f;
			vector<vector<Vector3f>> controlPoints = setSplineControlpoints(&Scalp, vertexList, normalList, startPoints, minY);
			vector<tinynurbs::Curve<float>> splineList = createSplines(&Scalp, controlPoints);

			// Tests
			TestStartPoints(&m_HeadTransformSGN, startPoints, vertexList, &m_Test);
			TestControlPoints(&m_HeadTransformSGN, controlPoints, &m_Test);
			TestSplines(&m_HeadTransformSGN, splineList, &m_Test);
			
			m_TestStartPointsGroupSGN.enable(false, false);
			m_TestControlPointsGroupSGN.enable(false, false);
			m_TestSplinesGroupSGN.enable(false, false);
			
			LineOfText* pKeybindings = new LineOfText();
			LineOfText* pSampleRate = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Show [Start Points: 1, Control Points: 2, Spline Sample Points: 3, Off: 0]");
			pSampleRate->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Spline Sample Rate (Shown Points) +/-: Up/Down");
			m_HelpTexts.push_back(pKeybindings);
			pKeybindings->color(0.0f, 0.0f, 0.0f, 1.0f);
			m_DrawHelpTexts = true;

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
			Keyboard* pKeyboard = m_RenderWin.keyboard();


			if (pKeyboard->keyPressed(Keyboard::KEY_1, true)) {
				m_TestStartPointsGroupSGN.enable(true, true);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_2, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(true, true);
				m_TestSplinesGroupSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_3, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(true, true);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_0, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(false, false);
			}
			/*
			if (pKeyboard->keyPressed(Keyboard::KEY_UP, true)) {
				if (sampleRate < 100.0f) {
					sampleRate += 5.0f;
					m_TestSplinesGroupSGN.clear()
					TestSplines
				}
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_DOWN, true)) {
				if (sampleRate > 10.0f) sampleRate -= 5.0f;
			}*/
			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			m_RenderDev.activeCamera(&m_Cam);
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
			// Skybox should be last thing to render
			m_SkyboxSG.render(&m_RenderDev);

			if (m_FPSLabelActive) m_FPSLabel.render(&m_RenderDev);
			if (m_DrawHelpTexts) drawHelpTexts();

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

		std::vector<SGNTransformation*> m_TestStartPointsTransformSGNs;
		std::vector<SGNGeometry*> m_TestStartPointsSGNs;
		SGNTransformation m_TestStartPointsGroupSGN;

		std::vector<SGNTransformation*> m_TestControlPointsTransformSGNs;
		std::vector<SGNGeometry*> m_TestControlPointsSGNs;
		SGNTransformation m_TestControlPointsGroupSGN;

		std::vector<SGNTransformation*> m_TestSplinesTransformSGNs;
		std::vector<SGNGeometry*> m_TestSplinesSGNs;
		SGNTransformation m_TestSplinesGroupSGN;

		static vector<Vector3f> vertexList;
		float sampleRate = 30.0f;

	};//HairModelGen

}//name space

#endif