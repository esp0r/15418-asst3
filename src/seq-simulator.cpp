#include "quad-tree.h"
#include "world.h"
#include <algorithm>
#include <iostream>

// TASK 1

// NOTE: You may modify any of the contents of this file, but preserve all
// function types and names. You may add new functions if you believe they will
// be helpful.

const int QuadTreeLeafSize = 8;
class SequentialNBodySimulator : public INBodySimulator {
public:
  std::unique_ptr<QuadTreeNode> buildQuadTree(std::vector<Particle> &particles,
                                              Vec2 bmin, Vec2 bmax) {
    // TODO: implement a function that builds and returns a quadtree containing
    // particles.
    if (particles.size() <= QuadTreeLeafSize) {
      auto node = std::make_unique<QuadTreeNode>();
      node->isLeaf = true;
      node->particles = particles;
      return node;
    } else {
      auto node = std::make_unique<QuadTreeNode>();
      node->isLeaf = false;
      Vec2 pivot = (bmin + bmax) * 0.5f;
      Vec2 size = (bmax - bmin) * 0.5f;
      for (int i = 0; i < 4; i++) {
        Vec2 childBMin;
        // 00: left bottom
        // 01: right bottom
        // 10: left top
        // 11: right top
        childBMin.x = (i & 1) ? pivot.x : bmin.x;
        childBMin.y = ((i >> 1) & 1) ? pivot.y : bmin.y;
        Vec2 childBMax = childBMin + size;
        std::vector<Particle> childParticles;
        for (auto &p : particles) {
          if (p.position.x >= childBMin.x && p.position.x < childBMax.x &&
            p.position.y >= childBMin.y && p.position.y < childBMax.y) {
            childParticles.push_back(p);
          }
        }
        node->children[i] = buildQuadTree(childParticles, childBMin, childBMax);
      }
      return node;
    }
  }
  virtual std::unique_ptr<AccelerationStructure>
  buildAccelerationStructure(std::vector<Particle> &particles) {
    // build quad-tree
    auto quadTree = std::make_unique<QuadTree>();

    // find bounds
    Vec2 bmin(1e30f, 1e30f);
    Vec2 bmax(-1e30f, -1e30f);

    for (auto &p : particles) {
      bmin.x = fminf(bmin.x, p.position.x);
      bmin.y = fminf(bmin.y, p.position.y);
      bmax.x = fmaxf(bmax.x, p.position.x);
      bmax.y = fmaxf(bmax.y, p.position.y);
    }

    quadTree->bmin = bmin;
    quadTree->bmax = bmax;

    // build nodes
    quadTree->root = buildQuadTree(particles, bmin, bmax);
    if (!quadTree->checkTree()) {
      std::cout << "Your Tree has Error!" << std::endl;
    }

    return quadTree;
  }
  virtual void simulateStep(AccelerationStructure *accel,
                            std::vector<Particle> &particles,
                            std::vector<Particle> &newParticles,
                            StepParameters params) override {
    // TODO: implement sequential version of quad-tree accelerated n-body
    // simulation here, using quadTree as acceleration structure
    // build quad-tree
    auto quadTree = buildAccelerationStructure(particles);
    for (int i = 0; i < (int)particles.size(); i++) {
      auto pi = particles[i];
      // get nearby particles
      auto nearbyParticles = std::vector<Particle>();
      quadTree->getParticles(nearbyParticles, pi.position, params.cullRadius);
      // accumulate attractive forces to apply to particle i
      Vec2 force = Vec2(0.0f, 0.0f);
      for (auto &p : nearbyParticles) {
        if (p.id == pi.id)
          continue;
        force += computeForce(pi, p, params.cullRadius);
      }
      // update particle state using the computed force
      newParticles[i] = updateParticle(pi, force, params.deltaTime);
    }
  }
};

std::unique_ptr<INBodySimulator> createSequentialNBodySimulator() {
  return std::make_unique<SequentialNBodySimulator>();
}
