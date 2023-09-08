#include "skinning.h"
#include "vec3d.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
  int isLBS = 1;
  
  vector<Vec4d> originalMeshVertexPositionBuffer;
  vector<Vec4d> newMeshVertexPositionBuffer;
  for (int j = 0; j < numMeshVertices; j++)
  {
    originalMeshVertexPositionBuffer.push_back(Vec4d(restMeshVertexPositions[3 * j + 0], restMeshVertexPositions[3 * j + 1], restMeshVertexPositions[3 * j + 2], 1.0));
    newMeshVertexPositionBuffer.push_back(Vec4d(0.0, 0.0, 0.0, 1.0));
  }
  if (isLBS == 1) {
    // v = sum joint weisht_i jointTrans_ji v_original
    for (int i = 0; i < numJointsInfluencingEachVertex; i++) {
      for (int j = 0; j < numMeshVertices; j++)
      {
        // Get Index
        int ind = j * numJointsInfluencingEachVertex + i;
        newMeshVertexPositionBuffer[j] += meshSkinningWeights[ind] * jointSkinTransforms[meshSkinningJoints[ind]] * originalMeshVertexPositionBuffer[j];
      }
    }
  }
  else 
  {
    //Dual Quaternion
    // not implemented 
    //for (int i = 0; i < numMeshVertices; i++)
    //{
    //  Eigen::Quaterniond qReal = { 0.0, 0.0, 0.0, 0.0 };
    //  Eigen::Quaterniond qDual = { 0.0, 0.0, 0.0, 0.0 };
    //  //int ind = j * numJointsInfluencingEachVertex + i;
    //}
  }
  
  // Update results
  for (int i = 0; i < numMeshVertices; i++)
  {
    newMeshVertexPositions[3 * i + 0] = newMeshVertexPositionBuffer[i][0];
    newMeshVertexPositions[3 * i + 1] = newMeshVertexPositionBuffer[i][1];
    newMeshVertexPositions[3 * i + 2] = newMeshVertexPositionBuffer[i][2];
  }
}

