#include <stdio.h>
#include <string.h>
#include <vector>

#include "catch2/catch_all.hpp"

#include "Recast.h"
#include "RecastAlloc.h"

#include <iostream>
#include <random>
#include <algorithm>
#include <iomanip>

namespace
{
	void freeHeightfieldSpans(const rcHeightfield& heightfield)
	{
		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				rcSpan* span = heightfield.spans[x + z * heightfield.width];
				while (span)
				{
					rcSpan* const prev = span;
					span = span->next;
					rcFree(prev);
				}
			}
		}
	}

	std::vector<unsigned> getAreas(const rcHeightfield& heightfield)
	{
		std::vector<unsigned> result(heightfield.height * heightfield.width);
		for (int i = 0; i < heightfield.height * heightfield.width; ++i)
			if (const rcSpan* const span = heightfield.spans[i])
				result[i] = span->area;
		return result;
	}
}

TEST_CASE("rcFilterLowHangingWalkableObstacles", "[recast, filtering]")
{
	rcContext context;
	int walkableHeight = 5;

	rcHeightfield heightfield;
	heightfield.width = 1;
	heightfield.height = 1;
	heightfield.bmin[0] = 0;
	heightfield.bmin[1] = 0;
	heightfield.bmin[2] = 0;
	heightfield.bmax[0] = 1;
	heightfield.bmax[1] = 1;
	heightfield.bmax[2] = 1;
	heightfield.cs = 1;
	heightfield.ch = 1;
	heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("Span with no spans above it is unchanged")
	{
		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);

		rcFree(span);
	}

	SECTION("Span with span above that is higher than walkableHeight is unchanged")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 1 + walkableHeight;
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that nothing has changed.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		// Check again but with a more clearance
		secondSpan->smin += 10;
		secondSpan->smax += 10;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that nothing has changed.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Marks low obstacles walkable if they're below the walkableClimb")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 1 + (walkableHeight - 1);
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that the second span was changed to walkable.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == 1);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Low obstacle that overlaps the walkableClimb distance is not changed")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 2 + (walkableHeight - 1);
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that the second span was changed to walkable.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Only the first of multiple, low obstacles are marked walkable")
	{
		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcSpan* previousSpan = span;
		for (int i = 0; i < 9; ++i)
		{
			rcSpan* nextSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
			nextSpan->area = RC_NULL_AREA;
			nextSpan->next = NULL;
			nextSpan->smin = previousSpan->smax + (walkableHeight - 1);
			nextSpan->smax = nextSpan->smin + 1;
			previousSpan->next = nextSpan;
			previousSpan = nextSpan;
		}

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		rcSpan* currentSpan = heightfield.spans[0];
		for (int i = 0; i < 10; ++i)
		{
			REQUIRE(currentSpan != NULL);
			// only the first and second spans should be marked as walkabl
			REQUIRE(currentSpan->area == (i <= 1 ? 1 : RC_NULL_AREA));
			currentSpan = currentSpan->next;
		}

		std::vector<rcSpan*> toFree;
		span = heightfield.spans[0];
		for (int i = 0; i < 10; ++i)
		{
			toFree.push_back(span);
			span = span->next;
		}

		for (int i = 0; i < 10; ++i)
		{
			rcFree(toFree[i]);
		}
	}
}

TEST_CASE("rcFilterLedgeSpans", "[recast, filtering]")
{
	SECTION("Edge spans are marked unwalkable")
	{
		rcContext context;
		const int walkableClimb = 5;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 10;
		heightfield.height = 10;
		heightfield.bmin[0] = 0;
		heightfield.bmin[1] = 0;
		heightfield.bmin[2] = 0;
		heightfield.bmax[0] = 10;
		heightfield.bmax[1] = 1;
		heightfield.bmax[2] = 10;
		heightfield.cs = 1;
		heightfield.ch = 1;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);
		heightfield.pools = NULL;
		heightfield.freelist = NULL;

		// Create a flat plane.
		for (int x = 0; x < heightfield.width; ++x)
		{
			for (int z = 0; z < heightfield.height; ++z)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->next = NULL;
				span->smin = 0;
				span->smax = 1;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		for (int x = 0; x < heightfield.width; ++x)
		{
			for (int z = 0; z < heightfield.height; ++z)
			{
				rcSpan* span = heightfield.spans[x + z * heightfield.width];
				REQUIRE(span != NULL);

				if (x == 0 || z == 0 || x == 9 || z == 9)
				{
					REQUIRE(span->area == RC_NULL_AREA);
				}
				else
				{
					REQUIRE(span->area == 1);
				}

				REQUIRE(span->next == NULL);
				REQUIRE(span->smin == 0);
				REQUIRE(span->smax == 1);
			}
		}

		// Free all the heightfield spans
		freeHeightfieldSpans(heightfield);
	}

	SECTION("Should skip unwalkable")
	{
		rcContext context;
		const int walkableClimb = 1;
		const int walkableHeight = 1;

		rcHeightfield heightfield;
		heightfield.width = 1;
		heightfield.height = 1;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = RC_NULL_AREA;
		span->smin = 0;
		span->smax = 0;
		span->next = NULL;
		heightfield.spans[0] = span;

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		CHECK(heightfield.spans[0]->area == 0);

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Too elevated span is marked unwalkable")
	{
		rcContext context;
		const int walkableClimb = 5;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 5;
		heightfield.height = 5;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->smin = 0;
				span->smax = 1;
				span->next = NULL;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		heightfield.spans[2 + 2 * heightfield.width]->smax = walkableClimb + 2;

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[2 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->area == 1);

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Too lowered span makes neighbours unwalkable")
	{
		rcContext context;
		const int walkableClimb = 5;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 5;
		heightfield.height = 5;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->smin = 0;
				span->smax = walkableClimb + 2;
				span->next = NULL;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		heightfield.spans[2 + 2 * heightfield.width]->smax = 1;

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[2 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 3 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->area == 1);

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Neighbour span at different depth keep layer walkable")
	{
		rcContext context;
		const int walkableClimb = 5;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 5;
		heightfield.height = 5;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				if (z == 2 && x == 2)
					continue;

				rcSpan* bottomSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				bottomSpan->area = 1;
				bottomSpan->smin = 0;
				bottomSpan->smax = 1;

				rcSpan* topSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				topSpan->area = 1;
				topSpan->smin = 1 + walkableHeight;
				topSpan->smax = 1 + walkableHeight + 1;
				topSpan->next = NULL;

				bottomSpan->next = topSpan;

				heightfield.spans[x + z * heightfield.width] = bottomSpan;
			}
		}

		{
			rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
			span->area = 1;
			span->smin = 0;
			span->smax = 1 + walkableHeight;
			span->next = NULL;

			heightfield.spans[2 + 2 * heightfield.width] = span;
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->area == 1);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->next->area == 1);

		CHECK(heightfield.spans[2 + 3 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->next->area == 1);

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Gradual stairs are walkable")
	{
		rcContext context;
		const int walkableClimb = 6;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 5;
		heightfield.height = 5;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->smin = 0;
				span->smax = 1 + z * (walkableClimb / 2);
				span->next = NULL;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->area == 1);

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Steep stairs are unwalkable")
	{
		rcContext context;
		const int walkableClimb = 6;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 5;
		heightfield.height = 5;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->smin = 0;
				span->smax = 1 + z * (walkableClimb / 2 + 1);
				span->next = NULL;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[2 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[2 + 3 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->area == 0);

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Neighbour span at different depth keeps layer walkable")
	{
		rcContext context;
		const int walkableClimb = 5;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 5;
		heightfield.height = 5;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				if (x == 1 && z == 2)
				{
					rcSpan* bottomSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
					bottomSpan->area = 1;
					bottomSpan->smin = 0;
					bottomSpan->smax = 1;

					rcSpan* topSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
					topSpan->area = 1;
					topSpan->smin = 40;
					topSpan->smax = 41;
					topSpan->next = NULL;

					bottomSpan->next = topSpan;

					heightfield.spans[x + z * heightfield.width] = bottomSpan;
				}
				else
				{
					rcSpan* bottomSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
					bottomSpan->area = 1;
					bottomSpan->smin = 20;
					bottomSpan->smax = 21;

					rcSpan* topSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
					topSpan->area = 1;
					topSpan->smin = 40;
					topSpan->smax = 41;
					topSpan->next = NULL;

					bottomSpan->next = topSpan;

					heightfield.spans[x + z * heightfield.width] = bottomSpan;
				}
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[2 + 2 * heightfield.width]->area == 0);
		CHECK(heightfield.spans[2 + 3 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->area == 1);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->area == 1);

		CHECK(heightfield.spans[1 + 1 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[1 + 2 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[1 + 3 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[2 + 1 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[2 + 2 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[2 + 3 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[3 + 1 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[3 + 2 * heightfield.width]->next->area == 1);
		CHECK(heightfield.spans[3 + 3 * heightfield.width]->next->area == 1);

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Random sample")
	{
		rcContext context;
		const int walkableClimb = 5;
		const int walkableHeight = 10;

		rcHeightfield heightfield;
		heightfield.width = 5;
		heightfield.height = 5;
		heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

		const unsigned smin[] = {
			0,  0,  0,  0,  0,
			0,  0, 11,  0,  0,
			0,  6,  0, 10,  0,
			0,  0, 11,  0,  0,
			0,  0,  0,  0,  0,
		};

		const unsigned smax[] = {
			1,  1,  1,  1,  1,
			1,  1, 12,  1,  1,
			1,  7,  1, 11,  1,
			1,  1, 12,  1,  1,
			1,  1,  1,  1,  1,
		};

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->smin = smin[x + z * heightfield.width];
				span->smax = smax[x + z * heightfield.width];
				span->next = NULL;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		const std::vector<unsigned> expectedAreas = {
			0, 0, 0, 0, 0,
			0, 1, 0, 1, 0,
			0, 0, 1, 0, 0,
			0, 1, 0, 1, 0,
			0, 0, 0, 0, 0,
		};

		CHECK_THAT(getAreas(heightfield), Catch::Matchers::Equals(expectedAreas));

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Random")
	{
		rcContext context;
		const int walkableClimb = 5;
		const int walkableHeight = 10;

		std::vector<std::size_t> spans(25);
		std::vector<std::vector<unsigned>> smin(25);
		std::vector<std::vector<unsigned>> smax(25);

		const auto generate = [&](rcHeightfield& heightfield) {
			heightfield.width = 5;
			heightfield.height = 5;
			heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);

			for (int i = 0; i < heightfield.width * heightfield.height; ++i)
				heightfield.spans[i] = nullptr;

			for (int z = 0; z < heightfield.height; ++z)
			{
				for (int x = 0; x < heightfield.width; ++x)
				{
					const int index = x + z * heightfield.width;
					rcSpan* prev = nullptr;

					for (std::size_t i = 0; i < spans[index]; ++i)
					{
						rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
						span->area = 1;
						span->smin = smin[index][spans[index] - i - 1];
						span->smax = smax[index][spans[index] - i - 1];
						span->next = prev;
						prev = span;

						if (heightfield.spans[x + z * heightfield.width] == nullptr)
							heightfield.spans[x + z * heightfield.width] = span;
					}
				}
			}
		};

		std::minstd_rand random;
		random.seed(1013904223);
		std::uniform_int_distribution<unsigned> coordinatesDistribution{0, 11};
		std::uniform_int_distribution<int> spansDistribution{1, 1};

		const std::pair<int, int> fill[] = {
			{1, 2},
			{2, 1},
			{2, 2},
			{2, 3},
			{3, 2},
		};

		for (int i = 0; i < 1000000; ++i)
		{
			for (int i = 0; i < 25; ++i)
				spans[i] = std::max(1, spansDistribution(random));

			for (int i = 0; i < 25; ++i)
			{
				if (smin[i].size() < spans[i])
					smin[i].resize(spans[i], 0);
				if (smax[i].size() < spans[i])
					smax[i].resize(spans[i], 1);
			}

			for (const auto [x, z] : fill)
			{
				const int i = x + z * 5;
				const std::size_t n = spans[i];
				for (std::size_t j = 0; j < n; ++j)
				{
					const unsigned min = coordinatesDistribution(random);
					smin[i][j] = min;
					smax[i][j] = std::max(min + 1, coordinatesDistribution(random));
				}
			}

			rcHeightfield heightfield1;
			generate(heightfield1);

			rcHeightfield heightfield2;
			generate(heightfield2);

			rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield1);
			rcFilterLedgeSpansNew(&context, walkableHeight, walkableClimb, heightfield2);

			unsigned diff = 0;
			unsigned maxDiff = 0;

			for (int z = 0; z < heightfield1.height; ++z)
			{
				for (int x = 0; x < heightfield1.width; ++x)
				{
					const int index = x + z * heightfield1.width;
					const rcSpan* span1 = heightfield1.spans[index];
					const rcSpan* span2 = heightfield2.spans[index];

					while (span1 && span2)
					{
						if (span1->area != span2->area)
							++diff;

						span1 = span1->next;
						span2 = span2->next;
					}
				}
			}

			freeHeightfieldSpans(heightfield1);
			freeHeightfieldSpans(heightfield2);

			if (maxDiff < diff)
				maxDiff = diff;

			if (diff >= maxDiff && diff > 4)
			{
				std::cout << "diff=" << diff << std::endl;

				const std::size_t maxSpans = *std::max_element(spans.begin(), spans.end());

				std::cout << "const std::size_t spans[] = {\n";
				for (int z = 0; z < heightfield1.height; ++z)
				{
					for (int x = 0; x < heightfield1.width; ++x)
						std::cout << std::setw(2) << std::setfill(' ') << spans[x + z * heightfield1.width] << ", ";
					std::cout << '\n';
				}
				std::cout << "};\n";

				std::cout << "const unsigned smin[][] = {\n";

				for (std::size_t i = 0; i < maxSpans; ++i)
				{
					std::cout << "{\n";
					for (int z = 0; z < heightfield1.height; ++z)
					{
						for (int x = 0; x < heightfield1.width; ++x)
							std::cout << std::setw(2) << std::setfill(' ') << smin[x + z * heightfield1.width][0] << ", ";
						std::cout << '\n';
					}
					std::cout << "},\n";
				}

				std::cout << "};\n";

				std::cout << "const unsigned smax[][] = {\n";

				for (std::size_t i = 0; i < maxSpans; ++i)
				{
					std::cout << "{\n";
					for (int z = 0; z < heightfield1.height; ++z)
					{
						for (int x = 0; x < heightfield1.width; ++x)
							std::cout << std::setw(2) << std::setfill(' ') << smax[x + z * heightfield1.width][0] << ", ";
						std::cout << '\n';
					}
					std::cout << "},\n";
				}

				std::cout << "};" << std::endl;

				break;
			}
		}
	}
}

TEST_CASE("rcFilterWalkableLowHeightSpans", "[recast, filtering]")
{
	rcContext context;
	int walkableHeight = 5;

	rcHeightfield heightfield;
	heightfield.width = 1;
	heightfield.height = 1;
	heightfield.bmin[0] = 0;
	heightfield.bmin[1] = 0;
	heightfield.bmin[2] = 0;
	heightfield.bmax[0] = 1;
	heightfield.bmax[1] = 1;
	heightfield.bmax[2] = 1;
	heightfield.cs = 1;
	heightfield.ch = 1;
	heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("span nothing above is unchanged")
	{
		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);

		rcFree(span);
	}

	SECTION("span with lots of room above is unchanged")
	{
		rcSpan* overheadSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		overheadSpan->area = RC_NULL_AREA;
		overheadSpan->next = NULL;
		overheadSpan->smin = 10;
		overheadSpan->smax = 11;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = overheadSpan;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(overheadSpan);
		rcFree(span);
	}

	SECTION("Span with low hanging obstacle is marked as unwalkable")
	{
		rcSpan* overheadSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		overheadSpan->area = RC_NULL_AREA;
		overheadSpan->next = NULL;
		overheadSpan->smin = 3;
		overheadSpan->smax = 4;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = overheadSpan;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == RC_NULL_AREA);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(overheadSpan);
		rcFree(span);
	}
}
