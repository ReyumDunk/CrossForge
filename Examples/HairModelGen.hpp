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

		bool VectorIsEqual(Vector3f v1, Vector3f v2) {
			if (!v1.x() == v2.x()) return false;
			if (!v1.y() == v2.y()) return false;
			if (!v1.z() == v2.z()) return false;
			return true;
		}

		glm::vec3 glmVecFromVector3f(Vector3f vector) {
			return glm::vec3(vector.x(), vector.y(), vector.z());
		}

		Vector3f Vector3fFromGlmVec3(glm::vec3 vector) {
			return Vector3f(vector.x, vector.y, vector.z);
		}

		Vector3f rotateVector3f(Vector3f vector, Vector3f rotationVector, float angle) {
			//normalized vector and rotation Vector
			Vector3f h3;
			Vector4f h;
			h = Vector4f(vector.x(), vector.y(), vector.z(), 1.0f);
			h = CForgeMath::rotationMatrix((Quaternionf)AngleAxisf(CForgeMath::degToRad(angle), rotationVector)) * h;
			h3 = Vector3f(h.x(), h.y(), h.z());
			return h3;
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

		vector<int> getStartpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f> &vertexList, vector<vector<int>> &adjacencyList, vector<double> weights, Vector2f partingXY, int pointNumber = 10) {
			//TODO change Vector3f vertex representation to index rep.
			vector<Vector3f> startPoints;
			//get all Vertices with x ~ partingX and y >= partingY
			vector<int> sampleVertices;

			float delta = 40.0f / scalp->vertexCount();		//tolerance around x
			//float delta = 0.1f;
			for (int i = 0; i < vertexList.size(); i++) {
				if (vertexList[i].y() >= (partingXY.y() - delta))
					if ((vertexList[i].x() >= (partingXY.x() - delta)) && (vertexList[i].x() <= (partingXY.x() + delta))) sampleVertices.push_back(i);
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
				igl::dijkstra(source, targets, adjacencyList, weights, min_d, previous);
				//printf("previous: %d\n", previous.size());
				igl::dijkstra(maxZ, previous, path);

				//TODO extract pointNumber of points from path with approx. even distance
				//TODO solution to to abnormal path on icosphere
				//TODO FaceMale solution
				
				//temporary solution to abnormal path on normal sphere
				sort(path.begin(), path.end(), CForge::HairModelGen::compareVectorZ);
				//path.erase(path.begin() + 5, path.end() - 5);
			}

			return path;
		}
		vector<int> getTESTStartpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f>& vertexList, Vector2f partingXY, int pointNumber = 10) {
			vector<int> startPoints;
			//get all Vertices with x ~ partingX and y >= partingY
			vector<int> sampleVertices;

			float delta = 150.0f / scalp->vertexCount();		//tolerance around x
			for (int i = 0; i < vertexList.size(); i++) {
				if (vertexList[i][1] >= (partingXY.y() - delta))
					if ((vertexList[i][0] >= (partingXY.x() - delta)) && (vertexList[i][0] <= (partingXY.x() + delta))) sampleVertices.push_back(i);
			}

			for (auto i : sampleVertices) {
				startPoints.push_back(i);
			}
			sort(startPoints.begin(), startPoints.end(), CForge::HairModelGen::compareVectorZ);
			//startPoints.erase(startPoints.begin() + 3, startPoints.end() - 3);

			//prevent multiple vertices in same position
			for (int i = 0; i < startPoints.size() - 1; i++) {
				if (vertexList[startPoints[i]] == vertexList[startPoints[i + 1]]) {
					startPoints.erase(startPoints.begin() + i + 1);
					i--;
				}
			}
			
			return startPoints;
		}
		
		vector<vector<Vector3f>> setSplineControlpoints(CForge::T3DMesh<float>* scalp, vector<Vector3f>& vertexList, vector<Vector3f>& normalList, vector<int> startPoints, Vector2f parting, float minY, vector<Vector3f>* sideVecs) {
			vector<vector<Vector3f>> controlPoints;
			sideVecs->clear();
			T3DMesh<float>::AABB scalpAABB = scalp->aabb();
			float ctrlPointNum = 4.0f;
			Vector3f middleStartPoint = vertexList[startPoints[(int)startPoints.size() / 2]];
			//approximate max length is double diagonal length between start and end point divided by sqrt(2) (3 controlpoints)
			//pythagoras on isosceles triangle: sqrt(2)*a = c -> 2*a = 2*c/sqrt(2)
			//additionally divided by number of desired controlpoints-1
			float vecLength = (Vector3f(scalpAABB.Max.x(), minY, 0.0f) - middleStartPoint).stableNorm();
			vecLength = 2.0f / 1.4f * vecLength / (ctrlPointNum - 1.1f);		//smaller value for controlPoint number so that it guarantees constant number in creation loop
			//Vector3f TestRotVec = Vector3f::UnitZ();							//TODO
			
			//left side
			float initialAngleLeft = -90.0f;
			float rotAngleLeft = (-180.0f - initialAngleLeft) / (ctrlPointNum - 2.0f);
			if (parting.x() > 0) rotAngleLeft += parting.x() * (-300.0f);		//TODO adjust angle added according to position
			else if (parting.x() < 0) {
				rotAngleLeft += parting.x() * (100.0f);
			}
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
				sideVecs->push_back(currentRotVec);
				//apply initial rotation to currentVec 
				currentVec = rotateVector3f(currentVec, currentRotVec, initialAngleLeft);
				//apply length to currentVec -> about 4(-6?) controlpoints
				currentVec.normalize();
				currentVec = vecLength * currentVec;
				//angle adjustment to height
				float adjustAngleLeft = -5.0f / currentPoint.y();

				//control point creation loop per Spline
				for (float i = 1.0f; i < ctrlPointNum - 1.0f; i += 1.0f) {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					//rotate currentVec, rotation angle depends on horizontal position of parting on scalp
					//currentRotVec += Vector3f::UnitZ();
					//currentRotVec.normalize();
					if (currentVec.x() > 0.0f) {
						currentVec = rotateVector3f(currentVec, currentRotVec, rotAngleLeft + adjustAngleLeft);
					}
					//prevent resulting vector from pointing in opposite x direction from before
					//TODO improve by using angle instead of 1 total direction
					if (currentVec.x() < 0.0f) currentVec = Vector3f(0.0f, currentVec.y(), currentVec.z());
					//apply gravity and then length
					//currentVec += Vector3f(0.0f, -0.05f, 0.0f);		//TODO adjust gravity value
					currentVec += Vector3f(0.0f, 0.0f, -currentVec.z() / 2);		//TODO adjust gravity value
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
			float initialAngleRight = 90.0f;
			float rotAngleRight = (180.0f - initialAngleRight) / (ctrlPointNum - 2.0f);
			if (parting.x() < 0) rotAngleRight += parting.x() * (-300.0f);		//TODO adjust angle added according to position
			else if (parting.x() > 0) {
				rotAngleRight += parting.x() * (100.0f);
			}
			
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
				sideVecs->push_back(-currentRotVec);
				//apply initial rotation to currentVec 
				currentVec.normalize();
				currentVec = rotateVector3f(currentVec, currentRotVec, initialAngleRight);
				//h = CForgeMath::rotationMatrix((Quaternionf)AngleAxisf(CForgeMath::degToRad(initialAngleRight), TestRotVec)) * h;
				//apply length to currentVec -> about 4(-6?) controlpoints
				currentVec.normalize();
				currentVec = vecLength * currentVec;
				//angle adjustment to height
				float adjustAngleRight = 5.0f / currentPoint.y();


				//control point creation loop per Spline
				for (float i = 1.0f; i < ctrlPointNum - 1.0f; i += 1.0f) {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					//rotate currentVec, rotation angle depends on horizontal position of parting on scalp
					//currentRotVec += Vector3f::UnitZ();
					//currentRotVec.normalize();
					if (currentVec.x() < 0.0f) {
						currentVec = rotateVector3f(currentVec, currentRotVec, rotAngleRight + adjustAngleRight);
					}
					//prevent resulting vector from pointing in opposite x direction from before
					//TODO improve by using angle instead of 1 total direction
					if (currentVec.x() > 0.0f) currentVec = Vector3f(0.0f, currentVec.y(), currentVec.z());
					//apply gravity and then length
					//currentVec += Vector3f(0.0f, -0.05f, 0.0f);		//TODO adjust gravity value
					currentVec += Vector3f(0.0f, 0.0f, -currentVec.z()/2);		//TODO adjust gravity value
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

			// back
			
			int stepSize = startPoints.size();
			Vector3f originNormal = normalList[startPoints[0]].normalized();
			Vector3f origin = vertexList[startPoints[0]];
			Vector3f currentRotVec = (vertexList[startPoints[1]] - origin).normalized();
			//TODO changing stepSize changes number of strips
			int stepSizeLeft = int(stepSize / 2);
			int stepSizeRight = stepSize - stepSizeLeft;
			float angleStepLeft = 90.0f / float(stepSizeLeft);
			float angleStepRight = 90.0f / float(stepSizeRight);
			currentRotVec = -1.0f * currentRotVec;
			rotAngleLeft = (rotAngleLeft - 5.0f * origin.y());
			rotAngleRight = -(rotAngleRight + 5.0f * origin.y());
			float rotAngleMiddle = min(rotAngleLeft, rotAngleRight) + 300.0f * origin.z();
			float vecLengthBack = (Vector3f(origin.x(), minY, scalpAABB.Min.z()) - origin).stableNorm();
			vecLengthBack = 2.0f / 1.4f * vecLengthBack / (ctrlPointNum - 1.1f);
			vecLengthBack = (vecLength + vecLengthBack) / 2.0f;
			//printf("%d %d %d\n", stepSize, stepSizeLeft, stepSizeRight);
			for (int i = 0; i < stepSize; i++) {
				vector<Vector3f> currentCtrlPts;
				Vector3f currentPoint = origin;
				currentCtrlPts.push_back(currentPoint);
				sideVecs->push_back(currentRotVec);
				Vector3f currentVec = rotateVector3f(originNormal, currentRotVec, 90.0f).normalized();
				currentVec *= vecLengthBack;

				//rotAngleBack interpolation between rotAngle Left-Middle or Right-Middle
				float rotAngleBack = rotAngleMiddle;
				//if (i <= stepSizeLeft) rotAngleBack = float(stepSizeLeft - i) / float(stepSizeLeft) * rotAngleLeft + float(i) / float(stepSizeLeft) * rotAngleMiddle;
				//else if (i > stepSizeLeft) rotAngleBack = float(i - stepSizeLeft) / float(stepSizeRight) * rotAngleRight + float(stepSizeRight - (i - stepSizeLeft)) / float(stepSizeRight) * rotAngleMiddle;

				//creation loop per spline
				for (float j = 1.0f; j < ctrlPointNum - 1.0f; j += 1.0f) {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					Vector3f h = rotateVector3f(currentVec, currentRotVec, rotAngleBack);
					//prevent resulting vector from pointing in opposite direction from before
					//TODO improve by using angle instead of 1 total direction
					if (h.z() > 0.0f) h = Vector3f(h.x(), h.y(), 0.0f);
					if ((h.x() > 0.0f && i <= stepSizeLeft ) || (h.x() < 0.0f && i > stepSizeLeft)) h = Vector3f(0.0f, h.y(), h.z());
					currentVec = h;
					//apply gravity and then length
					//currentVec += Vector3f(0.0f, -0.05f, 0.0f);		//TODO adjust gravity value
					currentVec += Vector3f(-currentVec.x() / 2, 0.0f, -currentVec.z() / 2);		//TODO adjust gravity value
					currentVec.normalize();
					currentVec = vecLengthBack * currentVec;
					//TODO IMPROVEMENT make vecLength proportional to number of current ctrl point so that earlier length is shorter

				}
				//last controlpoint is at minY
				Vector3f lastPoint = Vector3f(currentPoint.x(), minY, currentPoint.z());
				currentCtrlPts.push_back(lastPoint);
				controlPoints.push_back(currentCtrlPts);
				currentCtrlPts.clear();
				if (i <= stepSizeLeft) currentRotVec = rotateVector3f(currentRotVec, originNormal, angleStepLeft).normalized();
				else currentRotVec = rotateVector3f(currentRotVec, originNormal, angleStepRight).normalized();
			}
			
			return controlPoints;
		}

		void createFrontSplines() {
			//TODO
			return;
		}

		void createBackSplines() {
			//TODO
			return;
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
				spline.degree = 3;
				/*
				int knotNum = glmCtrlPts.size() + spline.degree + 1;
				for (int i = 1; i <= knotNum; i++) {
					if (i <= (int)(knotNum / 2)) knots.push_back(0.0f);
					else knots.push_back(1.0f);
				}*/
				
				for (int i = 0; i < spline.degree + 1; i++) knots.push_back(0.0f);
				for (int i = 0; i < glmCtrlPts.size(); i++) knots.push_back(1.0f);
				
				//knots = { 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
				spline.knots = knots;
				if (!tinynurbs::curveIsValid(spline)) {
					printf("Spline not valid\n");
				}
				splineList.push_back(spline);
			}
			return splineList;
		}

		void createStrips(T3DMesh<float>* pMesh, vector<int> startPoints, vector<Vector3f> vertexList, vector<tinynurbs::Curve<float>> splines, vector<Vector3f> sideVecs, float sampleRate) {
			 if (nullptr == pMesh) throw NullpointerExcept("pMesh");
			//vector<T3DMesh<float>> pMeshes;
			//strip width
			float width = 0.0f;
			float delta = 0.1f;
			vector<Vector3f> startVertices;
			for (int i = 0; i < startPoints.size(); i++) {
				startVertices.push_back(vertexList[startPoints[i]]);
			}
			for (int i = 0; i < startVertices.size(); i++) {
				float hWidth = 0.0f;
				if (i == 0) hWidth = (startVertices[i + 1] - startVertices[i]).stableNorm();
				else if (i == startVertices.size() - 1) hWidth = (startVertices[i] - startVertices[i - 1]).stableNorm();
				else hWidth = ((startVertices[i] - startVertices[i - 1]) + (startVertices[i + 1] - startVertices[i])).stableNorm() / 2.0f;
				if (hWidth > width) width = hWidth;
			}
			width += delta; width /= 2.0f;
			uint32_t pSampleRate = int(sampleRate);

			std::vector<Vector3f> Vertices;
			std::vector<Vector3f> UVWs;
			T3DMesh<float>::Submesh Sub;
			T3DMesh<float>::Material Mat;
			T3DMesh<float>::Face F;

			for (int i = 0; i < splines.size(); i++) {
				//T3DMesh<float> pMesh; pMesh.init();

				// generate vertices
				for (float j = 0.0f; j <= 1.0f; j += 1.0f / sampleRate) {
					Vector3f v1 = Vector3fFromGlmVec3(tinynurbs::curvePoint(splines[i], j)) + width * sideVecs[i];
					Vector3f v2 = Vector3fFromGlmVec3(tinynurbs::curvePoint(splines[i], j)) - width * sideVecs[i];
					Vertices.push_back(v1);
					Vertices.push_back(v2);
					Vector3f uvw1 = Vector3f(0.0f, 1.0f - j, 0.0f);
					Vector3f uvw2 = Vector3f(1.0f, 1.0f - j, 0.0f);
					UVWs.push_back(uvw1);
					UVWs.push_back(uvw2);
				}
				// generate triangles
				for (uint32_t j = 0; j < 2 * (pSampleRate - 1); j += 2) {
						auto v0 = j ;
						auto v1 = j + 1;
						auto v2 = j + 2;
						auto v3 = j + 3;
						F.Vertices[0] = v2;
						F.Vertices[2] = v0;
						F.Vertices[1] = v1;
						Sub.Faces.push_back(F);
						F.Vertices[0] = v2;
						F.Vertices[2] = v1;
						F.Vertices[1] = v3;
						Sub.Faces.push_back(F);
				}

				//pMeshes.push_back(pMesh);
				//pMesh.clear();
			}
			Mat.Color = Vector4f::Ones();
			Sub.Material = 0;
			pMesh->vertices(&Vertices);
			pMesh->textureCoordinates(&UVWs);
			pMesh->addSubmesh(&Sub, true);
			pMesh->addMaterial(&Mat, true);

			return;
		}
		/*
		vector<StaticActor> createStrips(vector<int> startPoints, vector<Vector3f> vertexList, vector<tinynurbs::Curve<float>> splines, vector<Vector3f> sideVecs, float sampleRate) {
			vector<StaticActor> pActors;
			//strip width
			float width = 0.0f;
			float delta = 0.0f;
			vector<Vector3f> startVertices;
			for (int i = 0; i < startPoints.size(); i++) {
				startVertices.push_back(vertexList[startPoints[i]]);
			}
			for (int i = 0; i < startVertices.size(); i++) {
				float hWidth = 0.0f;
				if (i == 0) hWidth = (startVertices[i + 1] - startVertices[i]).stableNorm();
				else if (i == startVertices.size() - 1) hWidth = (startVertices[i] - startVertices[i - 1]).stableNorm();
				else hWidth = ((startVertices[i] - startVertices[i - 1]) + (startVertices[i + 1] - startVertices[i])).stableNorm() / 2.0f;
				if (hWidth > width) width = hWidth;
			}
			width += delta; width /= 2.0f;


			for (int i = 0; i < splines.size(); i++) {
				T3DMesh<float> pMesh;
				StaticActor pActor;
				createStrip(&pMesh, width, splines[i], sampleRate, sideVecs[i]);
				pActor.init(&pMesh);
				pActors.push_back(pActor);
				//pMesh.clear();
			}

			return pActors;
		}
		void createStrip(T3DMesh<float>* pMesh, float width, tinynurbs::Curve<float> spline, float sampleRate, Vector3f sideVec) {
			if (nullptr == pMesh) throw NullpointerExcept("pMesh");
			pMesh->clear();
			uint32_t pSampleRate = int(sampleRate);

			std::vector<Vector3f> Vertices;
			std::vector<Vector3f> UVWs;
			T3DMesh<float>::Submesh Sub;
			T3DMesh<float>::Material Mat;
			T3DMesh<float>::Face F;

			// generate vertices
			for (float j = 0.0f; j <= 1.0f; j += 1.0f / sampleRate) {
				Vector3f v1 = Vector3fFromGlmVec3(tinynurbs::curvePoint(spline, j)) + width * sideVec;
				Vector3f v2 = Vector3fFromGlmVec3(tinynurbs::curvePoint(spline, j)) - width * sideVec;
				Vertices.push_back(v1);
				Vertices.push_back(v2);
				Vector3f uvw1 = Vector3f(0.0f, 1.0f - j, 0.0f);
				Vector3f uvw2 = Vector3f(1.0f, 1.0f - j, 0.0f);
				UVWs.push_back(uvw1);
				UVWs.push_back(uvw2);
			}
			// generate triangles
			for (uint32_t j = 0; j < 2 * (pSampleRate - 1); j += 2) {
				auto v0 = j;
				auto v1 = j + 1;
				auto v2 = j + 2;
				auto v3 = j + 3;
				F.Vertices[0] = v2;
				F.Vertices[2] = v0;
				F.Vertices[1] = v1;
				Sub.Faces.push_back(F);
				F.Vertices[0] = v2;
				F.Vertices[2] = v1;
				F.Vertices[1] = v3;
				Sub.Faces.push_back(F);
			}
			Mat.Color = Vector4f::Ones();
			Sub.Material = 0;
			pMesh->vertices(&Vertices);
			pMesh->textureCoordinates(&UVWs);
			pMesh->addSubmesh(&Sub, true);
			pMesh->addMaterial(&Mat, true);

			return;
		}
		void TestStrips(SGNTransformation* headTransformSGN, vector<StaticActor>* testActors) {
		//void TestStrips(SGNTransformation* headTransformSGN, T3DMesh<float>* strips, StaticActor* testActor) {
			m_TestStripsGroupSGN.init(headTransformSGN);
			
			for (auto i : *testActors) {
				// create the scene graph nodes
				SGNGeometry* pGeomSGN = nullptr;
				SGNTransformation* pTransformSGN = nullptr;

				// initialize position and scaling
				pTransformSGN = new SGNTransformation();
				pTransformSGN->init(&m_TestStripsGroupSGN);

				// initialize geometry
				pGeomSGN = new SGNGeometry();
				pGeomSGN->init(pTransformSGN, &i);

				m_TestStripsTransformSGNs.push_back(pTransformSGN);
				m_TestStripsSGNs.push_back(pGeomSGN);
			}//for[stripNumber]

			//m_TestStripsTransformSGN.init(headTransformSGN);
			//testActor->init(strips);
			//m_TestStripsSGN.init(&m_TestStripsTransformSGN, testActor);
		}*/

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
				for (float j = 0.0f; j <= 1.0f; j+=1.0f/20.0f) {
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

		//void TestStrips(SGNTransformation* headTransformSGN, vector<T3DMesh<float>> strips, vector<StaticActor>* testActors) {
		void TestStrips(SGNTransformation* headTransformSGN, T3DMesh<float>* strips, StaticActor* testActor) {
			//m_TestStripsGroupSGN.init(headTransformSGN);
			/*
			for (auto i : strips) {
				StaticActor pActor;
				pActor.init(&i);
				testActors->push_back(pActor);
			}
			for (auto i : *testActors) {
				// create the scene graph nodes
				SGNGeometry* pGeomSGN = nullptr;
				SGNTransformation* pTransformSGN = nullptr;

				// initialize position and scaling
				pTransformSGN = new SGNTransformation();
				pTransformSGN->init(&m_TestStripsGroupSGN);

				// initialize geometry
				pGeomSGN = new SGNGeometry();
				pGeomSGN->init(pTransformSGN, &i);

				m_TestStripsTransformSGNs.push_back(pTransformSGN);
				m_TestStripsSGNs.push_back(pGeomSGN);
			}//for[stripNumber]*/

			m_TestStripsTransformSGN.init(headTransformSGN);
			testActor->init(strips);
			m_TestStripsSGN.init(&m_TestStripsTransformSGN, testActor);
		}

		void init() override{

			initWindowAndRenderDevice();
			initCameraAndLights();

			// load skydome and a scalp
			T3DMesh<float> M, Scalp, TestM;
			
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

			SAssetIO::load("MyAssets/sphere.obj", &TestM);
			setMeshShader(&TestM, 0.5f, 0.04f);
			TestM.computePerVertexNormals();
			//SAssetIO::load("MyAssets/icosphere.obj", &Scalp);
			SAssetIO::load("MyAssets/MaleFace.obj", &Scalp);
			setMeshShader(&Scalp, 0.5f, 0.04f);
			Scalp.computePerVertexNormals();
			m_Scalp.init(&Scalp);

			//testing
			//set Color for Mesh 
			for (uint32_t i = 0; i < TestM.materialCount(); ++i) {
				T3DMesh<float>::Material* pMat = TestM.getMaterial(i);
				pMat->Color = Vector4f(1.0f, 0.0f, 0.0f, 0.6f);
			}
			m_Test.init(&TestM);

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
			Vector2f partingXY = Vector2f(0.2f, 0.6f);
			vector<double> weights;
			int stripNumber = 10;
			for (int i = 0; i < Scalp.vertexCount(); i++) {
				vertexList.push_back(Scalp.vertex(i));
				//weights.push_back(1.0f);
				weights.push_back(1.0f - abs(Scalp.vertex(i).y()));
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
			//startPoints = getStartpoints(&Scalp, vertexList, adjList, weights, partingXY, stripNumber);
			startPoints = getTESTStartpoints(&Scalp, vertexList, partingXY, stripNumber);
			//printf("%d \n", startPoints.size());
			vector <Vector3f> normalList;
			for (int i = 0; i < Scalp.normalCount(); i++) {
				normalList.push_back(Scalp.normal(i));
			}
			//stop height for hair
			float minY = -0.5f;
			vector<Vector3f> sideVecs;
			vector<vector<Vector3f>> controlPoints = setSplineControlpoints(&Scalp, vertexList, normalList, startPoints, partingXY, minY, &sideVecs);
			vector<tinynurbs::Curve<float>> splineList = createSplines(&Scalp, controlPoints);
			T3DMesh<float> pMesh;
			float sampleRate = 10.0f;
			createStrips(&pMesh, startPoints, vertexList, splineList, sideVecs, sampleRate);

			// Tests
			//for (auto i : startPoints) {
				//printf("%.2f %.2f %.2f \n", normalList[i].x(), normalList[i].y(), normalList[i].z());
				//printf("%.2f %.2f %.2f \n", vertexList[i].x(), vertexList[i].y(), vertexList[i].z());
			//}
			TestStartPoints(&m_HeadTransformSGN, startPoints, vertexList, &m_Test);
			TestControlPoints(&m_HeadTransformSGN, controlPoints, &m_Test);
			TestSplines(&m_HeadTransformSGN, splineList, &m_Test);
			TestStrips(&m_HeadTransformSGN, &pMesh, &m_TestStrip);
			//PrimitiveShapeFactory::plane(&pMesh, Vector2f(10.0f, 10.0f), Vector2i(20, 20));
			//m_TestStripsTransformSGN.init(&m_HeadTransformSGN);
			//m_TestStrip.init(&pMesh);
			//m_TestStripsSGN.init(&m_TestStripsTransformSGN, &m_TestStrip);

			m_TestStartPointsGroupSGN.enable(false, false);
			m_TestControlPointsGroupSGN.enable(false, false);
			m_TestSplinesGroupSGN.enable(false, false);
			//m_TestStripsGroupSGN.enable(false, false);
			m_TestStripsTransformSGN.enable(false, false);

			LineOfText* pKeybindings = new LineOfText();
			LineOfText* pSampleRate = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Show [Start Points: 1, Control Points: 2, Spline Sample Points: 3, Strips: 4, Off: 0]");
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
				//m_TestStripsGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_2, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(true, true);
				m_TestSplinesGroupSGN.enable(false, false);
				//m_TestStripsGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_3, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(true, true);
				//m_TestStripsGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_4, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(false, false);
				//m_TestStripsGroupSGN.enable(true, true);
				m_TestStripsTransformSGN.enable(true, true);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_0, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(false, false);
				//m_TestStripsGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(false, false);
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

		vector<SGNTransformation*> m_TestStartPointsTransformSGNs;
		vector<SGNGeometry*> m_TestStartPointsSGNs;
		SGNTransformation m_TestStartPointsGroupSGN;

		vector<SGNTransformation*> m_TestControlPointsTransformSGNs;
		vector<SGNGeometry*> m_TestControlPointsSGNs;
		SGNTransformation m_TestControlPointsGroupSGN;

		vector<SGNTransformation*> m_TestSplinesTransformSGNs;
		vector<SGNGeometry*> m_TestSplinesSGNs;
		SGNTransformation m_TestSplinesGroupSGN;

		//vector<StaticActor> m_StripList;
		//vector<SGNTransformation*> m_TestStripsTransformSGNs;
		//vector<SGNGeometry*> m_TestStripsSGNs;
		//SGNTransformation m_TestStripsGroupSGN;

		StaticActor m_TestStrip;
		SGNTransformation m_TestStripsTransformSGN;
		SGNGeometry m_TestStripsSGN;

		static vector<Vector3f> vertexList;
		//float sampleRate = 30.0f;

	};//HairModelGen

}//name space

#endif