// Package recast defines functions and types to create mesh data that is then used
// to create Detour navigation meshes.
//
// The are a large number of possible ways to building navigation mesh data.
// One of the simplest piplines is as follows:
//
//  - Prepare the input triangle mesh.
//  - Build a Heightfield.
//  - Build a CompactHeightfield.
//  - Build a ContourSet.
//  - Build a PolyMesh.
//  - Build a PolyMeshDetail.
//  - Use the PolyMesh and PolyMeshDetail to build a Detour navigation mesh
//   tile.
//
// The general life-cycle of the main classes is as follows:
//
//  - Initialize or build the object. (E.g. BuildPolyMesh)
//  - Update the object as needed. (E.g. RasterizeTriangles)
//  - Use the object as part of the pipeline.
//  - Free the object if it has a Free() function and if it remains in scope but
//    won't be used. (E.g. FreeHeightField)
package recast
