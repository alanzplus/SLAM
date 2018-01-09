#include "pose_graph.hpp"
#include "gtest/gtest.h"

#include <memory>
#include <iostream>
#include <Eigen/Core>

using namespace rgbdslam;
using namespace std;

class PoseGraphTest : public ::testing::Test {
 protected:
  PoseGraphTest() {
    PoseGraph::Create();
    graph_ = PoseGraph::instance();
  }

  virtual ~PoseGraphTest() {

  }

  virtual void SetUp() {

  }

  virtual void TearDown() {
    graph_->ClearVertices();
  }

  PoseGraph* graph_;

};

TEST_F(PoseGraphTest, TestSaveLoadVertex) {
  VertexId id1 = 1;
  Eigen::Matrix4d m1;
  m1 <<  1, 2, 3, 4.5,
        2.3, 4, 5, 6,
        8, 9, 10, 11,
        12, 13, 14, 15;
  Vertex v1(id1, m1);
  graph_->InsertVertex(v1);
  graph_->SaveVertexById(id1, "1.v");
  Vertex vv1;
  EXPECT_TRUE(graph_->LoadVertexById("1.v", id1, &vv1));
  EXPECT_EQ(m1, vv1.pose);

  VertexId id2 = 2;
  Eigen::Matrix4d m2;
  m2 << 4.5, 3, 0, 0,
        12, 49, 23, 33,
        45, 67, 89, 99,
        1230, 23, 11, 0.1;
  Vertex v2(id2, m2);
  graph_->InsertVertex(v2);
  graph_->SaveVertexById(id2, "1.v");
  Vertex vv2;
  EXPECT_TRUE(graph_->LoadVertexById("1.v", id2, &vv2));
  EXPECT_EQ(m2, vv2.pose);

  EXPECT_TRUE(!graph_->LoadVertexById("1.v", id1 + id2, &vv2));
}

TEST_F(PoseGraphTest, TestSaveLoadVertices) {
  VertexId id1 = 1;
  Eigen::Matrix4d m1;
  m1 <<  1, 2, 3, 4.5,
        2.3, 4, 5, 6,
        8, 9, 10, 11,
        12, 13, 14, 15;
  Vertex v1(id1, m1);
  graph_->InsertVertex(v1);

  VertexId id2 = 2;
  Eigen::Matrix4d m2;
  m2 << 4.5, 3, 0, 0,
        12, 49, 23, 33,
        45, 67, 89, 99,
        1230, 23, 11, 0.1;
  Vertex v2(id2, m2);
  graph_->InsertVertex(v2);

  graph_->SaveVertices("v");
  graph_->ClearVertices();
  graph_->LoadVertices("v");

  auto& vv1 = graph_->GetVertexById(id1);
  EXPECT_EQ(vv1.id, v1.id);
  EXPECT_EQ(vv1.pose, v1.pose);

  auto& vv2 = graph_->GetVertexById(id2);
  EXPECT_EQ(vv2.id, v2.id);
  EXPECT_EQ(vv2.pose, v2.pose);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}