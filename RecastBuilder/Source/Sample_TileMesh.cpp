//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TileMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


inline unsigned int nextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}


Sample_TileMesh::Sample_TileMesh() :
	m_keepInterResults(false),
	m_buildAll(true),
	m_totalBuildTimeMs(0),
	m_triareas(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_pmesh(0),
	m_dmesh(0),
	m_drawMode(DRAWMODE_NAVMESH),
	m_maxTiles(0),
	m_maxPolysPerTile(0),
	m_tileSize(256),
	m_tileCol(duRGBA(0,0,0,32)),
	m_tileBuildTime(0),
	m_tileMemUsage(0),
	m_tileTriCount(0)
{
	resetCommonSettings();
	memset(m_lastBuiltTileBmin, 0, sizeof(m_lastBuiltTileBmin));
	memset(m_lastBuiltTileBmax, 0, sizeof(m_lastBuiltTileBmax));
}

Sample_TileMesh::~Sample_TileMesh()
{
	cleanup();
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
}

void Sample_TileMesh::cleanup()
{
	delete [] m_triareas;
	m_triareas = 0;
	rcFreeHeightField(m_solid);
	m_solid = 0;
	rcFreeCompactHeightfield(m_chf);
	m_chf = 0;
	rcFreeContourSet(m_cset);
	m_cset = 0;
	rcFreePolyMesh(m_pmesh);
	m_pmesh = 0;
	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = 0;
}

void Sample_TileMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (m_geom)
	{
		int gw = 0, gh = 0;
		const float* bmin = m_geom->getNavMeshBoundsMin();
		const float* bmax = m_geom->getNavMeshBoundsMax();
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		const int ts = (int)m_tileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;
		printf("Tiles %d x %d\n", tw, th);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
		printf("Max Tiles %d\n", m_maxTiles);
		printf("Max Polys %d\n", m_maxPolysPerTile);
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}
}

void Sample_TileMesh::handleTools()
{
	if (m_tool)
		m_tool->handleMenu();
}

void Sample_TileMesh::handleDebugMode()
{
}

void Sample_TileMesh::handleRender()
{
}

void Sample_TileMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
    (void)proj; // Explicitly mark proj as unused
    (void)model; // Explicitly mark model as unused
    (void)view; // Explicitly mark view as unused

}

void Sample_TileMesh::handleMeshChanged(InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	const BuildSettings* buildSettings = geom->getBuildSettings();
	if (buildSettings && buildSettings->tileSize > 0)
		m_tileSize = buildSettings->tileSize;

	cleanup();

	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_TileMesh::handleBuild()
{
	if (!m_geom || !m_geom->getMesh())
	{
		printf("buildTiledNavigation: No vertices and triangles.\n");
		return false;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		printf("buildTiledNavigation: Could not allocate navmesh.\n");
		return false;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, m_geom->getNavMeshBoundsMin());
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;

	dtStatus status;

	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		printf("buildTiledNavigation: Could not init navmesh.\n");
		return false;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		printf("buildTiledNavigation: Could not init Detour navmesh query\n");
		return false;
	}

	if (m_buildAll)
		buildAllTiles();

	if (m_tool)
		m_tool->init(this);
	initToolStates(this);

	return true;
}

void Sample_TileMesh::collectSettings(BuildSettings& settings)
{
	Sample::collectSettings(settings);

	settings.tileSize = m_tileSize;
}

void Sample_TileMesh::buildTile(const float* pos)
{
	if (!m_geom) return;
	if (!m_navMesh) return;

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();

	const float ts = m_tileSize*m_cellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);

	m_lastBuiltTileBmin[0] = bmin[0] + tx*ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty*ts;

	m_lastBuiltTileBmax[0] = bmin[0] + (tx+1)*ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty+1)*ts;

	m_tileCol = duRGBA(255,255,255,64);

	m_ctx->resetLog();

	int dataSize = 0;
	bool canBeIgnored;
	unsigned char* data = buildTileMesh(tx, ty, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize, canBeIgnored);

	// Remove any previous data (navmesh owns and deletes the data).
	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx,ty,0),0,0);

	// Add tile, or leave the location empty.
	if (data)
	{
		// Let the navmesh own the data.
		dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
			dtFree(data);
	}

	m_ctx->dumpLog("Build Tile (%d,%d):", tx,ty);
}

void Sample_TileMesh::getTilePos(const float* pos, int& tx, int& ty)
{
	if (!m_geom) return;

	const float* bmin = m_geom->getNavMeshBoundsMin();

	const float ts = m_tileSize*m_cellSize;
	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}

void Sample_TileMesh::removeTile(const float* pos)
{
	if (!m_geom) return;
	if (!m_navMesh) return;

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();

	const float ts = m_tileSize*m_cellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);

	m_lastBuiltTileBmin[0] = bmin[0] + tx*ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty*ts;

	m_lastBuiltTileBmax[0] = bmin[0] + (tx+1)*ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty+1)*ts;

	m_tileCol = duRGBA(128,32,16,64);

	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx,ty,0),0,0);
}

void Sample_TileMesh::buildAllTiles()
{
	if (!m_geom) return;
	if (!m_navMesh) return;

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	const float tcs = m_tileSize*m_cellSize;


	// Start the build process.
	int TilesAdded = 0;
	int TilesIgnored = 0;
	m_ctx->startTimer(RC_TIMER_TEMP);
	for (int y = 0; y < th; ++y)
	{
		int TilesAddedInRow = 0;
		int TilesIgnoredInRow = 0;
		printf("TileRow %d/%d: ", y, th);
		for (int x = 0; x < tw; ++x)
		{
			m_lastBuiltTileBmin[0] = bmin[0] + x*tcs;
			m_lastBuiltTileBmin[1] = bmin[1];
			m_lastBuiltTileBmin[2] = bmin[2] + y*tcs;

			m_lastBuiltTileBmax[0] = bmin[0] + (x+1)*tcs;
			m_lastBuiltTileBmax[1] = bmax[1];
			m_lastBuiltTileBmax[2] = bmin[2] + (y+1)*tcs;

			int dataSize = 0;
			bool canBeIgnored = false;
			unsigned char* data = buildTileMesh(x, y, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize, canBeIgnored);
			if (data && !canBeIgnored)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				m_navMesh->removeTile(m_navMesh->getTileRefAt(x,y,0),0,0);
				// Let the navmesh own the data.
				dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
				{
					printf("	Tile %d,%d failed with status: %0x\n", x, y, static_cast<unsigned int>(status));
					dtFree(data);
				}
				else
				{
					++TilesAddedInRow;
					++TilesAdded;
				}
			}
			else
			{
				++TilesIgnoredInRow;
				++TilesIgnored;
			}
		}
		printf("%d tiles added (total %d)\n", TilesAddedInRow, TilesAdded);
	}
	printf("Total tiles added: (%d / %d max)\n", TilesAdded, m_maxTiles);

	// Start the build process.
	m_ctx->stopTimer(RC_TIMER_TEMP);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TEMP)/1000.0f;

}

void Sample_TileMesh::removeAllTiles()
{
	if (!m_geom || !m_navMesh)
		return;

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;

	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			m_navMesh->removeTile(m_navMesh->getTileRefAt(x,y,0),0,0);
}


unsigned char* Sample_TileMesh::buildTileMesh(const int tx, const int ty, const float* bmin, const float* bmax, int& dataSize, bool &canBeIgnored)
{
	if (!m_geom || !m_geom->getMesh() || !m_geom->getChunkyMesh())
	{
		printf("buildNavigation: Input mesh is not specified.\n");
		return 0;
	}

	m_tileMemUsage = 0;
	m_tileBuildTime = 0;

	cleanup();

	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const rcChunkyTriMesh* chunkyMesh = m_geom->getChunkyMesh();

	// Init build configuration from GUI
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.tileSize = (int)m_tileSize;
	m_cfg.borderSize = m_cfg.walkableRadius + 3; // Reserve enough padding.
	m_cfg.width = m_cfg.tileSize + m_cfg.borderSize*2;
	m_cfg.height = m_cfg.tileSize + m_cfg.borderSize*2;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
	//
	// This is done in order to make sure that the navmesh tiles connect correctly at the borders,
	// and the obstacles close to the border work correctly with the dilation process.
	// No polygons (or contours) will be created on the border area.
	//
	// IMPORTANT!
	//
	//   :''''''''':
	//   : +-----+ :
	//   : |     | :
	//   : |     |<--- tile to build
	//   : |     | :
	//   : +-----+ :<-- geometry needed
	//   :.........:
	//
	// You should use this bounding box to query your input geometry.
	//
	// For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
	// you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8 neighbours,
	// or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
	rcVcopy(m_cfg.bmin, bmin);
	rcVcopy(m_cfg.bmax, bmax);
	m_cfg.bmin[0] -= m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmin[2] -= m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmax[0] += m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmax[2] += m_cfg.borderSize*m_cfg.cs;

	// Reset build times gathering.
	m_ctx->resetTimers();

	// Start the build process.
	m_ctx->startTimer(RC_TIMER_TOTAL);

	// printf("Building navigation:\n");
	// printf(" - %d x %d cells\n", m_cfg.width, m_cfg.height);
	// printf(" - %.1fK verts, %.1fK tris\n", nverts/1000.0f, ntris/1000.0f);

	// Allocate voxel heightfield where we rasterize our input data to.
	m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		printf("buildNavigation: Out of memory 'solid'.\n");
		return 0;
	}
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		printf("buildNavigation: Could not create solid heightfield.\n");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!m_triareas)
	{
		printf("buildNavigation: Out of memory 'm_triareas' (%d).\n", chunkyMesh->maxTrisPerChunk);
		return 0;
	}

	float tbmin[2], tbmax[2];
	tbmin[0] = m_cfg.bmin[0];
	tbmin[1] = m_cfg.bmin[2];
	tbmax[0] = m_cfg.bmax[0];
	tbmax[1] = m_cfg.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0;
	}

	m_tileTriCount = 0;

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* ctris = &chunkyMesh->tris[node.i*3];
		const int nctris = node.n;

		m_tileTriCount += nctris;

		memset(m_triareas, 0, nctris*sizeof(unsigned char));
		rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle,
								verts, nverts, ctris, nctris, m_triareas);

		if (!rcRasterizeTriangles(m_ctx, verts, nverts, ctris, m_triareas, nctris, *m_solid, m_cfg.walkableClimb))
		{
			return 0;
		}
	}

	if (!m_keepInterResults)
	{
		delete [] m_triareas;
		m_triareas = 0;
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		printf("buildNavigation: Out of memory 'chf'.\n");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		printf("buildNavigation: Could not build compact data.\n");
		return 0;
	}

	if (!m_keepInterResults)
	{
		rcFreeHeightField(m_solid);
		m_solid = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
	{
		printf("buildNavigation: Could not erode.\n");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	if (m_geom->getApplyMaterialAsType())
	{
		const char *mats = m_geom->getMesh()->getMats();
		const int *tri = m_geom->getMesh()->getTris();
		const float *verts = m_geom->getMesh()->getVerts();
		float faceVerts[3*3];
		float min, max;
		int ind;
		// printf("Defining areas by material\n\n");
		for (int i=0; i<m_geom->getMesh()->getTriCount(); ++i)
		{
			if (mats[i] != 0)
			{
				//printf("Defining area for %d\n", mats[i]);
				ind = tri[i*3] * 3;
				faceVerts[0] = verts[ind];
				faceVerts[1] = verts[ind+1];
				faceVerts[2] = verts[ind+2];
				ind = tri[i*3+1] * 3;
				faceVerts[3] = verts[ind];
				faceVerts[4] = verts[ind+1];
				faceVerts[5] = verts[ind+2];
				ind = tri[i*3+2] * 3;
				faceVerts[6] = verts[ind];
				faceVerts[7] = verts[ind+1];
				faceVerts[8] = verts[ind+2];
				// get min and max
				min = rcMin<float>(rcMin<float>(faceVerts[1], faceVerts[4]), faceVerts[7]);
				max = min +5;
				rcMarkConvexPolyArea(m_ctx, faceVerts, 3, min, max, mats[i], *m_chf);
			}
		}
	}

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_ctx, *m_chf))
		{
			printf("buildNavigation: Could not build distance field.\n");
			return 0;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			printf("buildNavigation: Could not build watershed regions.\n");
			return 0;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			printf("buildNavigation: Could not build monotone regions.\n");
			return 0;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea))
		{
			printf("buildNavigation: Could not build layer regions.\n");
			return 0;
		}
	}

	// Create contours.
	m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		printf("buildNavigation: Out of memory 'cset'.\n");
		return 0;
	}
	if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		printf("buildNavigation: Could not create contours.\n");
		return 0;
	}

	if (m_cset->nconts == 0)
	{
		return 0;
	}

	// Build polygon navmesh from the contours.
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		printf("buildNavigation: Out of memory 'pmesh'.\n");
		return 0;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		printf("buildNavigation: Could not triangulate contours.\n");
		return 0;
	}

	// Build detail mesh.
	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		printf("buildNavigation: Out of memory 'dmesh'.\n");
		return 0;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf,
							   m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
							   *m_dmesh))
	{
		printf("buildNavigation: Could build polymesh detail.\n");
		return 0;
	}

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	unsigned char* navData = 0;
	int navDataSize = 0;
	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (m_pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			printf("Too many vertices per tile %d (max: %d).\n", m_pmesh->nverts, 0xffff);
			return 0;
		}

		// Update poly flags from areas.
		canBeIgnored = true;
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == CARLA_AREA_SIDEWALK)
			{
				m_pmesh->flags[i] = CARLA_TYPE_SIDEWALK;
				canBeIgnored = false;
			}
			else if (m_pmesh->areas[i] == CARLA_AREA_CROSSWALK)
			{
				m_pmesh->flags[i] = CARLA_TYPE_CROSSWALK;
				canBeIgnored = false;
			}
			else if (m_pmesh->areas[i] == CARLA_AREA_GRASS)
			{
				m_pmesh->flags[i] = CARLA_TYPE_GRASS;
				canBeIgnored = false;
			}
			else if (m_pmesh->areas[i] == CARLA_AREA_ROAD)
			{
				m_pmesh->flags[i] = CARLA_TYPE_ROAD;
				canBeIgnored = false;
			}
			else
			{
				m_pmesh->flags[i] = CARLA_TYPE_NONE;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		params.tileX = tx;
		params.tileY = ty;
		params.tileLayer = 0;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			printf("Could not build Detour navmesh.\n");
			return 0;
		}
	}
	m_tileMemUsage = navDataSize/1024.0f;

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	// printf("%3d vertices, %3d polygons\n", m_pmesh->nverts, m_pmesh->npolys);

	m_tileBuildTime = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;

	dataSize = navDataSize;
	return navData;
}
