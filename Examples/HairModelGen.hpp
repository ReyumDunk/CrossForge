/*****************************************************************************\
*                                                                           *
* File(s): HairModelGen.hpp													*
*                                                                           *
* Content: Automated generation of hair strip models using parting position *
*          and hair stop height as parameter.					            *
*                                                                           *
*                                                                           *
* Author(s): Max Meyer	                                                    *
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

		struct simpleVertex {
			Vector3f pos;
			Vector3f normal;
		};

		typedef enum haircolor {
			BROWN = 0,
			BLONDE,
			RED
		};

		static bool compareVectorZ_I(const int &v1, const int &v2) {
			return (vertexList[v1].z() < vertexList[v2].z());
		}
		static bool compareVectorZ_simpleV(const simpleVertex& v1, const simpleVertex& v2) {
			return (v1.pos.z() < v2.pos.z());
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

		bool randomBool()
		{
			return 0 + (rand() % (1 - 0 + 1)) == 1;
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

		//Usage of Dijkstra-Algorithm to find source points for hair strips
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
			sort(sampleVertices.begin(), sampleVertices.end(), CForge::HairModelGen::compareVectorZ_I);
			if (sampleVertices.size() > 0) {
				minZ = sampleVertices[0];
				maxZ = sampleVertices[sampleVertices.size() - 1];
			}

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
			}
			return path;
		}

		//hair strip source points with simple interval tests
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
			sort(startPoints.begin(), startPoints.end(), CForge::HairModelGen::compareVectorZ_I);
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

		vector<simpleVertex> buildStartpoints(vector<int>& startPoints_I, vector<Vector3f>& vertexList, vector<Vector3f>& normalList) {
			vector<simpleVertex> startPoints;
			for (auto i : startPoints_I) {
				simpleVertex startPoint;
				startPoint.pos = vertexList[i];
				startPoint.normal = normalList[i];
				startPoints.push_back(startPoint);
			}
			return startPoints;
		}

		//generate extra startpoints as interpolation between 2 adjacent existing ones
		//	to factor correlated number of new points between every startpoint pair (factor = 2 -> double points -> 1 new point each)
		vector<simpleVertex> generateExtraStartpointsF(vector<simpleVertex>& startPoints) {
			vector<simpleVertex> updatedList;
			for (auto i : startPoints) updatedList.push_back(i);
			for (int i = 0; i < startPoints.size() - 1; i++) {
				for (int j = 1; j < factor; j++) {
					simpleVertex newPoint;
					float p_t = float(j) / float(factor);
					newPoint.pos = p_t * startPoints[i + 1].pos + (1.0f - p_t) * startPoints[i].pos;
					newPoint.normal = p_t * startPoints[i + 1].normal + (1.0f - p_t) * startPoints[i].normal;
					updatedList.push_back(newPoint);
				}
			}
			sort(updatedList.begin(), updatedList.end(), CForge::HairModelGen::compareVectorZ_simpleV);
			return updatedList;
		}
		//	only add for wider distances between pairs
		vector<int> generateExtraStartpoints(vector<int>& startPoints) {

		}

		vector<vector<Vector3f>> setSplineControlpoints(CForge::T3DMesh<float>* scalp, vector<simpleVertex> startPoints, Vector2f parting, float minY, vector<Vector3f>* sideVecs) {
			vector<vector<Vector3f>> controlPoints;
			sideVecs->clear();
			T3DMesh<float>::AABB scalpAABB = scalp->aabb();
			
			Vector3f middleStartPoint = startPoints[(int)startPoints.size() / 2].pos;
			//approximate max length is double diagonal length between start and end point divided by sqrt(2) (3 controlpoints)
			//pythagoras on isosceles triangle: sqrt(2)*a = c -> 2*a = 2*c/sqrt(2)
			//additionally divided by number of desired controlpoints-1
			float vecLength = (Vector3f(scalpAABB.Max.x(), minY, 0.0f) - middleStartPoint).stableNorm();
			//concentrate controlpoints around the scalp for more precision, necessary for smaller number of points
			vecLength = 2.0f / 1.4f * vecLength / (ctrlPointNum - 1.0f - 0.1f*ctrlPointNum*ctrlPointNum*(minY + 0.6f));		
			
			//left side
			float initialAngleLeft = -85.0f;
			float rotAngleLeft = (-180.0f - initialAngleLeft) / (ctrlPointNum - 2.0f) - 60.0f/(ctrlPointNum - 2.0f);
			
			for (int i = 0; i < startPoints.size(); i++) {
				Vector3f currentPoint = startPoints[i].pos;
				vector<Vector3f> currentCtrlPts;
				currentCtrlPts.push_back(currentPoint);
				//get normal from startPoints
				Vector3f currentVec = startPoints[i].normal;
				//get initial rotation vector

				Vector3f currentRotVec;
				if (i == 0) {
					currentRotVec = startPoints[i + 1].pos - currentPoint;
				}
				else if (i == startPoints.size() - 1) {
					currentRotVec = currentPoint - startPoints[i - 1].pos;
				}
				else {
					currentRotVec = currentPoint - startPoints[i - 1].pos + startPoints[i + 1].pos - currentPoint;
				}
				currentRotVec.normalize();
				sideVecs->push_back(currentRotVec);
				//apply initial rotation to currentVec 
				currentVec = rotateVector3f(currentVec, currentRotVec, initialAngleLeft);
				//apply length to currentVec
				currentVec.normalize();
				currentVec = vecLength * currentVec;
				//angle adjustment to height
				float adjustAngleLeft = -10.0f / currentPoint.y() / (ctrlPointNum -2.0f);

				//control point creation loop per Spline
				for (float i = 1.0f; i < ctrlPointNum - 1.0f; i += 1.0f) {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					//rotate currentVec
					
					if (currentVec.x() > 0.0f) {
						currentVec = rotateVector3f(currentVec, currentRotVec, rotAngleLeft + adjustAngleLeft);
					}
					//prevent resulting vector from pointing in opposite x direction from before
					//TODO improve by using angle instead of 1 total direction
					if (currentVec.x() < 0.0f) currentVec = Vector3f(0.0f, currentVec.y(), currentVec.z());
					//apply gravity and then length
					currentVec += Vector3f(0.0f, 0.0f, -currentVec.z() / 2);		//TODO adjust gravity value
					currentVec.normalize();
					currentVec = vecLength * currentVec;

				}
				//last controlpoint is at minY
				Vector3f lastPoint = Vector3f(currentPoint.x(), minY, currentPoint.z());
				currentCtrlPts.push_back(lastPoint);
				controlPoints.push_back(currentCtrlPts);
				currentCtrlPts.clear();
			}
			
			//right side same as left side only diffrent rotation angles and negative x TODO
			float initialAngleRight = 85.0f;
			float rotAngleRight = (180.0f - initialAngleRight) / (ctrlPointNum - 2.0f) + 60.0f / (ctrlPointNum - 2.0f);
			
			for (int i = 0; i < startPoints.size(); i++) {
				Vector3f currentPoint = startPoints[i].pos;
				vector<Vector3f> currentCtrlPts;
				currentCtrlPts.push_back(currentPoint);
				//get normal from startPoints
				Vector3f currentVec = startPoints[i].normal;
				//get initial rotation vector
				
				Vector3f currentRotVec;
				if (i == 0) {
					currentRotVec = startPoints[i + 1].pos - currentPoint;
				}
				else if (i == startPoints.size() - 1) {
					currentRotVec = currentPoint - startPoints[i - 1].pos;
				}
				else {
					currentRotVec = currentPoint - startPoints[i - 1].pos + startPoints[i + 1].pos - currentPoint;
				}
				currentRotVec.normalize();
				sideVecs->push_back(-currentRotVec);
				//apply initial rotation to currentVec 
				currentVec.normalize();
				currentVec = rotateVector3f(currentVec, currentRotVec, initialAngleRight);
				//apply length to currentVec
				currentVec.normalize();
				currentVec = vecLength * currentVec;
				//angle adjustment to height
				float adjustAngleRight = 10.0f / currentPoint.y() / (ctrlPointNum - 2.0f);


				//control point creation loop per Spline
				for (float i = 1.0f; i < ctrlPointNum - 1.0f; i += 1.0f) {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					//rotate currentVec
					if (currentVec.x() < 0.0f) {
						currentVec = rotateVector3f(currentVec, currentRotVec, rotAngleRight + adjustAngleRight);
					}
					//prevent resulting vector from pointing in opposite x direction from before
					//TODO improve by using angle instead of 1 total direction
					if (currentVec.x() > 0.0f) currentVec = Vector3f(0.0f, currentVec.y(), currentVec.z());
					//apply gravity and then length
					currentVec += Vector3f(0.0f, 0.0f, -currentVec.z()/2);		//TODO adjust gravity value
					currentVec.normalize();
					currentVec = vecLength * currentVec;

				}
				//last controlpoint is at minY
				Vector3f lastPoint = Vector3f(currentPoint.x(), minY, currentPoint.z());
				currentCtrlPts.push_back(lastPoint);
				controlPoints.push_back(currentCtrlPts);
				currentCtrlPts.clear();
			}

			// back
			
			int stepSize = int(startPoints.size() * 3 / 2);
			Vector3f originNormal = startPoints[0].normal.normalized();
			Vector3f origin = startPoints[0].pos;
			Vector3f currentRotVec = (startPoints[1].pos - origin).normalized();
			//TODO changing stepSize changes number of strips
			int stepSizeLeft = int(stepSize / 2);
			int stepSizeRight = stepSize - stepSizeLeft;
			float angleStepLeft = 90.0f / float(stepSizeLeft);
			float angleStepRight = 90.0f / float(stepSizeRight);
			currentRotVec = -1.0f * currentRotVec;
			rotAngleLeft = -(rotAngleLeft - 10.0f * origin.y() / (ctrlPointNum - 2.0f));
			rotAngleRight = (rotAngleRight + -10.0f * origin.y() / (ctrlPointNum - 2.0f));
			float rotAngleMiddle = max(rotAngleLeft, rotAngleRight);
			float vecLengthBack = (Vector3f(origin.x(), minY, scalpAABB.Min.z()) - origin).stableNorm();
			vecLengthBack = 2.0f / 1.4f * vecLengthBack / (ctrlPointNum - 1.0f - 0.1f * ctrlPointNum * ctrlPointNum * (minY + 0.5f));
			vecLengthBack = (vecLength + vecLengthBack) / 2.0f;
			for (int i = 0; i < stepSize; i++) {
				vector<Vector3f> currentCtrlPts;
				Vector3f currentPoint = origin;
				currentCtrlPts.push_back(currentPoint);
				sideVecs->push_back(currentRotVec);
				Vector3f currentVec = rotateVector3f(originNormal, currentRotVec, 89.0f).normalized();
				currentVec *= vecLengthBack;

				//rotAngleBack interpolation between rotAngle Left-Middle or Right-Middle
				float rotAngleBack = rotAngleMiddle;
				
				//creation loop per spline
				for (float j = 1.0f; j < ctrlPointNum - 1.0f; j += 1.0f) {
					//add currentVec to currentPoint
					currentPoint += currentVec;
					//push currentPoint to currentCtrlPts				
					currentCtrlPts.push_back(currentPoint);
					currentVec.normalize();
					
					Vector3f h = rotateVector3f(currentVec, currentRotVec, rotAngleBack);
					if (i < int(stepSizeLeft / 2) - 1 || i > (stepSizeLeft + int(stepSizeRight / 2) + 1)) {
						//prevent resulting vector from pointing in opposite direction from before
						//TODO improve by using angle instead of 1 total direction
						if (h.z() > 0.0f) h = Vector3f(h.x(), h.y(), 0.0f);
						if ((h.x() < 0.0f && i < stepSizeLeft) || (h.x() > 0.0f && i > stepSizeLeft)) h = Vector3f(0.0f, h.y(), h.z());
						currentVec = h;
					}
					else if (currentVec.x() != 0.0f || currentVec.z() != 0.0f) {
						float x = 1.0f;
						if (i < stepSizeLeft) x = float(i - int(stepSizeLeft / 2)) + 1.0f;
						else x = float(stepSizeLeft + int(stepSizeRight / 2) - i) + 1.0f;
						if (h.z() > 0.0f) h = Vector3f(h.x()/x, h.y(), 0.0f);
						if ((h.x() < 0.0f && i < stepSizeLeft) || (h.x() > 0.0f && i > stepSizeLeft)) h = Vector3f(0.0f, h.y(), h.z()/x);
						currentVec = h;

					}
					//apply gravity and then length
					currentVec = Vector3f(currentVec.x() / 2.0f, currentVec.y(), currentVec.z() / 2.0f);		//TODO adjust gravity value
					if (currentVec.y() > 0.0f) currentVec = Vector3f(currentVec.x(), -4.0f * currentVec.y(), currentVec.z());
					currentVec.normalize();
					currentVec = vecLengthBack * currentVec;
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
				spline.degree = glmCtrlPts.size() - 1;
				
				//bezier curve knots
				for (int i = 0; i < spline.degree + 1; i++) knots.push_back(0.0f);
				for (int i = 0; i < glmCtrlPts.size(); i++) knots.push_back(1.0f);

				/*
				//uniform spline knotvector
				for (int i = 1; i < spline.degree + glmCtrlPts.size() + 1; i++) {
					if (i <= spline.degree) knots.push_back(0.0f);
					else if (i <= glmCtrlPts.size() + 1) knots.push_back(float(i - spline.degree));
					else knots.push_back(float(glmCtrlPts.size() - spline.degree + 1));
				}
				/*
				float h = 0.0f;
				for (auto i : knots) {
					if (i > h) h = i;
				}
				for (int i = 0; i < knots.size(); i++) {
					knots[i] = knots[i] / h;
				}*/

				spline.knots = knots;
				if (!tinynurbs::curveIsValid(spline)) {
					printf("Spline not valid\n");
				}
				splineList.push_back(spline);
			}
			return splineList;
		}

		void createStrips(T3DMesh<float>* pMesh, vector<simpleVertex> startPoints, vector<Vector3f> vertexList, vector<tinynurbs::Curve<float>> splines, vector<Vector3f> sideVecs, float sampleRate) {
			 if (nullptr == pMesh) throw NullpointerExcept("pMesh");
			//strip width
			float width = 0.0f;
			vector<Vector3f> startVertices;
			for (int i = 0; i < startPoints.size(); i++) {
				startVertices.push_back(startPoints[i].pos);
			}
			for (int i = 0; i < startVertices.size(); i++) {
				float hWidth = 0.0f;
				if (i == 0) hWidth = (startVertices[i + 1] - startVertices[i]).stableNorm();
				else if (i == startVertices.size() - 1) hWidth = (startVertices[i] - startVertices[i - 1]).stableNorm();
				else hWidth = ((startVertices[i] - startVertices[i - 1]) + (startVertices[i + 1] - startVertices[i])).stableNorm() / 2.0f;
				if (hWidth > width) width = hWidth;
			}
			width += widthDelta; width /= 2.0f;
			uint32_t pSampleRate = int(sampleRate);

			std::vector<Vector3f> Vertices;
			std::vector<Vector3f> UVWs;
			T3DMesh<float>::Submesh Sub;
			T3DMesh<float>::Material Mat;
			T3DMesh<float>::Face F;

			bool useWaves = randomBool();
			for (int i = 0; i < splines.size(); i++) {
				// generate vertices
				float waveFactor = 0.0f;
				for (float j = 0.0f; j <= 1.0f; j += 1.0f / sampleRate) {
					if (useWaves) {
						waveFactor += (2.0f * float(rand()) / float(RAND_MAX) - 1.0f) / 20.0f;
						if (waveFactor < -0.05f) waveFactor = -0.05f;
						else if (waveFactor > 0.05f) waveFactor = 0.05f;
					}
					Vector3f v1 = Vector3fFromGlmVec3(tinynurbs::curvePoint(splines[i], j)) + (waveFactor + width) * sideVecs[i];
					Vector3f v2 = Vector3fFromGlmVec3(tinynurbs::curvePoint(splines[i], j)) + (waveFactor - width) * sideVecs[i];
					Vertices.push_back(v1);
					Vertices.push_back(v2);
					Vector3f uvw1 = Vector3f(0.0f, 1.0f - j, 0.0f);
					Vector3f uvw2 = Vector3f(1.0f, 1.0f - j, 0.0f);
					UVWs.push_back(uvw1);
					UVWs.push_back(uvw2);
				}
				// generate triangles
				for (uint32_t j = 0; j < 2 * (pSampleRate - 1); j += 2) {
						auto v0 = j + 2 * i * pSampleRate;
						auto v1 = j + 1 + 2 * i * pSampleRate;
						auto v2 = j + 2 + 2 * i * pSampleRate;
						auto v3 = j + 3 + 2 * i * pSampleRate;
						F.Vertices[0] = v2;
						F.Vertices[2] = v0;
						F.Vertices[1] = v1;
						Sub.Faces.push_back(F);
						F.Vertices[0] = v2;
						F.Vertices[2] = v1;
						F.Vertices[1] = v3;
						Sub.Faces.push_back(F);
				}
			}
			//hair color
			if (color == BROWN) Mat.TexAlbedo = "Assets/Hairgen/hair_texture2.webp";
			else if (color == BLONDE) Mat.TexAlbedo = "Assets/Hairgen/hair_texture2blonde.webp";
			else if (color == RED) Mat.TexAlbedo = "Assets/Hairgen/hair_texture2red.webp";
			Sub.Material = 0;
			pMesh->vertices(&Vertices);
			pMesh->textureCoordinates(&UVWs);
			pMesh->addMaterial(&Mat, true);
			pMesh->addSubmesh(&Sub, true);
			pMesh->computePerVertexNormals();
			

			return;
		}

		//visualizing the components
		void TestStartPoints(SGNTransformation* headTransformSGN, vector<simpleVertex> startPoints, StaticActor* testActor) {
			
			m_TestStartPointsGroupSGN.init(headTransformSGN);

			for (uint32_t i = 0; i < startPoints.size(); i++) {
				// create the scene graph nodes
				SGNGeometry* pGeomSGN = nullptr;
				SGNTransformation* pTransformSGN = nullptr;

				// initialize position and scaling
				pTransformSGN = new SGNTransformation();
				pTransformSGN->init(&m_TestStartPointsGroupSGN);

				pTransformSGN->translation(startPoints[i].pos);
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

		void TestStrips(SGNTransformation* headTransformSGN, T3DMesh<float>* strips, StaticActor* testActor) {
			m_TestStripsTransformSGN.init(headTransformSGN);
			testActor->init(strips);
			m_TestStripsSGN.init(&m_TestStripsTransformSGN, testActor);
		}

		virtual void initCameraAndLights(bool CastShadows = false) {
			// initialize camera
			m_Cam.init(Vector3f(0.0f, 3.0f, 8.0f), Vector3f::UnitY());
			m_Cam.projectionMatrix(m_WinWidth, m_WinHeight, CForgeMath::degToRad(45.0f), 0.1f, 1000.0f);

			// initialize sun (key light) and back ground light (fill light)
			Vector3f SunPos = Vector3f(-5.0f, 15.0f, 35.0f);
			Vector3f BGLightPos = Vector3f(0.0f, 5.0f, -30.0f);
			m_Sun.init(SunPos, -SunPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 5.0f);
			// sun will cast shadows

			//m_Sun.initShadowCasting(1024, 1024, GraphicsUtility::orthographicProjection(10.0f, 10.0f, 0.1f, 1000.0f));
			if (CastShadows) m_Sun.initShadowCasting(1024, 1024, Vector2i(10, 10), 0.1f, 1000.0f);
			m_BGLight.init(BGLightPos, -BGLightPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 1.5f, Vector3f(0.0f, 0.0f, 0.0f));

			// set camera and lights
			m_RenderDev.activeCamera(&m_Cam);
			m_RenderDev.addLight(&m_Sun);
			m_RenderDev.addLight(&m_BGLight);
		}//initCameraAndLights

		void init() override{

			initWindowAndRenderDevice();
			initCameraAndLights();
			srand(static_cast <unsigned> (time(0)));

			// load skydome and a scalp
			T3DMesh<float> M, Scalp, TestM;
			
			/// gather textures for the skyboxes
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_right.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_left.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_up.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_down.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_back.png");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/vz_clear_front.png");

			m_Skybox.init(m_ClearSky[0], m_ClearSky[1], m_ClearSky[2], m_ClearSky[3], m_ClearSky[4], m_ClearSky[5]);

			SAssetIO::load("Assets/Hairgen/sphere.obj", &TestM);
			setMeshShader(&TestM, 0.5f, 0.04f);
			TestM.computePerVertexNormals();
			//SAssetIO::load("Assets/Hairgen/icosphere.obj", &Scalp);
			SAssetIO::load("Assets/Hairgen/MaleFace.obj", &Scalp);
			setMeshShader(&Scalp, 0.5f, 0.04f);
			Scalp.computePerVertexNormals();
			m_Scalp.init(&Scalp);

			//testing sphere to visualize components
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


			//generating hair model
			// start points
			vector <int> startPoints;
			vector<double> weights;
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

			//Dijkstra ver
			//startPoints = getStartpoints(&Scalp, vertexList, adjList, weights, partingXY, stripNumber);

			//interval ver
			startPoints = getTESTStartpoints(&Scalp, vertexList, partingXY, stripNumber);

			// control points
			vector <Vector3f> normalList;
			for (int i = 0; i < Scalp.normalCount(); i++) {
				normalList.push_back(Scalp.normal(i));
			}
			vector<simpleVertex> startPointsList = buildStartpoints(startPoints, vertexList, normalList);
			startPointsList = generateExtraStartpointsF(startPointsList);
			vector<Vector3f> sideVecs;
			vector<vector<Vector3f>> controlPoints = setSplineControlpoints(&Scalp, startPointsList, partingXY, minY, &sideVecs);
			// splines
			vector<tinynurbs::Curve<float>> splineList = createSplines(&Scalp, controlPoints);
			T3DMesh<float> pMesh;
			// strips
			createStrips(&pMesh, startPointsList, vertexList, splineList, sideVecs, sampleRate);

			//visualization
			TestStartPoints(&m_HeadTransformSGN, startPointsList, &m_Test);
			TestControlPoints(&m_HeadTransformSGN, controlPoints, &m_Test);
			TestSplines(&m_HeadTransformSGN, splineList, &m_Test);
			TestStrips(&m_HeadTransformSGN, &pMesh, &m_TestStrip);

			//disabled to show one at a time
			m_TestStartPointsGroupSGN.enable(false, false);
			m_TestControlPointsGroupSGN.enable(false, false);
			m_TestSplinesGroupSGN.enable(false, false);
			m_TestStripsTransformSGN.enable(false, false);

			LineOfText* pKeybindings = new LineOfText();
			LineOfText* pSampleRate = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Show [Start Points: 1, Control Points: 2, Spline Sample Points: 3, Strips: 4, Off: 0]");
			pSampleRate->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Spline Sample Rate (Shown Points) +/-: Up/Down");
			m_HelpTexts.push_back(pKeybindings);
			pKeybindings->color(0.0f, 0.0f, 0.0f, 1.0f);
			m_DrawHelpTexts = true;
			gladLoadGL();

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

			//enable component
			if (pKeyboard->keyPressed(Keyboard::KEY_1, true)) {
				m_TestStartPointsGroupSGN.enable(true, true);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_2, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(true, true);
				m_TestSplinesGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_3, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(true, true);
				m_TestStripsTransformSGN.enable(false, false);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_4, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(true, true);
			}
			if (pKeyboard->keyPressed(Keyboard::KEY_0, true)) {
				m_TestStartPointsGroupSGN.enable(false, false);
				m_TestControlPointsGroupSGN.enable(false, false);
				m_TestSplinesGroupSGN.enable(false, false);
				m_TestStripsTransformSGN.enable(false, false);
			}
			
			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
			glDisable(GL_CULL_FACE);
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

		StaticActor m_TestStrip;
		SGNTransformation m_TestStripsTransformSGN;
		SGNGeometry m_TestStripsSGN;

		static vector<Vector3f> vertexList;


		//parameters
		int stripNumber = 10;			//not used currently
		int factor = 2;					//increase number of startpoints by factor
		float ctrlPointNum = 4.0f;		//scaling of curve
		haircolor color = RED;		//usage of multiple textures
		//Vector2f partingXY = Vector2f(0.2f, 0.6f);
		Vector2f partingXY = Vector2f(0.2f, 0.6f);		//possible x: {0.0f, 0.1f, 0.2f, 0.3f} mirrored to negative
		float minY = -0.5f; 
		float sampleRate = 20.0f;		//scaling of strips
		float widthDelta = 0.2f;		//added strip width on top of longest distance between startpoints
	};//HairModelGen

}//name space

#endif