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

#include "Recast.h"
#include "RecastAssert.h"

#include <stdlib.h>

#include <iostream>

void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			rcSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				const bool walkable = span->area != RC_NULL_AREA;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if (!walkable && previousWasWalkable)
				{
					if (rcAbs((int)span->smax - (int)previousSpan->smax) <= walkableClimb)
					{
						span->area = previousArea;
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWasWalkable = walkable;
				previousArea = span->area;
			}
		}
	}
}

void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb,
						rcHeightfield& heightfield, bool log)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
	
	// Mark border spans.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const bool debug = (x == 2) && (z == 2) && log;

			if (debug)
				std::cout << "x=" << x << " z=" << z << std::endl;

			int n = 0;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

				const int bot = (int)(span->smax);
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;

				if (debug)
					std::cout << "  n=" << n << " bot=" << bot << " top=" << top << std::endl;

				++n;

				// Find neighbours minimum height.
				int minNeighborHeight = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				int accessibleNeighborMinHeight = span->smax;
				int accessibleNeighborMaxHeight = span->smax;

				for (int direction = 0; direction < 4; ++direction)
				{
					int dx = x + rcGetDirOffsetX(direction);
					int dy = z + rcGetDirOffsetY(direction);

					if (debug)
						std::cout << "    direction=" << direction << " dx=" << dx << " dy=" << dy << std::endl;

					// Skip neighbours which are out of bounds.
					if (dx < 0 || dy < 0 || dx >= xSize || dy >= zSize)
					{
						minNeighborHeight = rcMin(minNeighborHeight, -walkableClimb - bot);
						if (debug)
							std::cout << "    out of bounds minNeighborHeight=" << minNeighborHeight << std::endl;
						continue;
					}

					// From minus infinity to the first span.
					const rcSpan* neighborSpan = heightfield.spans[dx + dy * xSize];
					int neighborBot = -walkableClimb;
					int neighborTop = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHT;

					if (debug)
						std::cout << "    neighborBot=" << neighborBot << " neighborTop=" << neighborTop
								  << " rcMin(top, neighborTop)=" << rcMin(top, neighborTop)
								  << " rcMax(bot, neighborBot)=" << rcMax(bot, neighborBot)
								  << " d=" << rcMin(top, neighborTop) - rcMax(bot, neighborBot) << std::endl;
					
					// Skip neighbour if the gap between the spans is too small.
					if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
					{
						minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);
						if (debug)
							std::cout << "    minNeighborHeight=" << minNeighborHeight << std::endl;
					}

					int m = 0;

					// Rest of the spans.
					for (neighborSpan = heightfield.spans[dx + dy * xSize]; neighborSpan; neighborSpan = neighborSpan->next)
					{
						neighborBot = (int)neighborSpan->smax;
						neighborTop = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHT;

						if (debug)
							std::cout << "      m=" << m++
									  << " neighborBot=" << neighborBot << " neighborTop=" << neighborTop
									  << " rcMin(top, neighborTop)=" << rcMin(top, neighborTop)
									  << " rcMax(bot, neighborBot)=" << rcMax(bot, neighborBot)
									  << " d=" << rcMin(top, neighborTop) - rcMax(bot, neighborBot)
									  << std::endl;
						
						// Skip neighbour if the gap between the spans is too small.
						if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
						{
							minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);

							// Find min/max accessible neighbour height. 
							if (rcAbs(neighborBot - bot) <= walkableClimb)
							{
								if (neighborBot < accessibleNeighborMinHeight)
								{
									accessibleNeighborMinHeight = neighborBot;
									if (debug)
										std::cout << "      accessibleNeighborMinHeight=" << accessibleNeighborMinHeight << std::endl;
								}

								if (neighborBot > accessibleNeighborMaxHeight)
								{
									accessibleNeighborMaxHeight = neighborBot;
									if (debug)
										std::cout << "      accessibleNeighborMaxHeight=" << accessibleNeighborMaxHeight << std::endl;
								}
							}

						}
					}
				}

				if (debug)
					std::cout << "  end minNeighborHeight=" << minNeighborHeight
							  << " accessibleNeighborMaxHeight=" << accessibleNeighborMaxHeight
							  << " accessibleNeighborMinHeight=" << accessibleNeighborMinHeight
							  << " d=" << (accessibleNeighborMaxHeight - accessibleNeighborMinHeight)
							  << std::endl;

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				if (minNeighborHeight < -walkableClimb)
				{
					if (debug)
						std::cout << "  RC_NULL_AREA minNeighborHeight" << std::endl;
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				else if ((accessibleNeighborMaxHeight - accessibleNeighborMinHeight) > walkableClimb)
				{
					if (debug)
						std::cout << "  RC_NULL_AREA accessibleNeighborMaxHeight" << std::endl;
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterLedgeSpansNew(rcContext* context, const int walkableHeight, const int walkableClimb,
						   rcHeightfield& heightfield, bool log)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.

	// Mark border spans.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const bool debug = (x == 2) && (z == 2) && log;

			if (debug)
				std::cout << "x=" << x << " z=" << z << std::endl;

			int n = 0;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

				const int bot = (int)(span->smax);
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;

				if (debug)
					std::cout << "  n=" << n << " bot=" << bot << " top=" << top << std::endl;

				++n;

				// Find neighbours minimum height.
				int minNeighborHeight = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				int accessibleNeighborMinHeight = span->smax;
				int accessibleNeighborMaxHeight = span->smax;

				for (int direction = 0; direction < 4; ++direction)
				{
					int dx = x + rcGetDirOffsetX(direction);
					int dz = z + rcGetDirOffsetY(direction);

					if (debug)
						std::cout << "    direction=" << direction << " dx=" << dx << " dz=" << dz << std::endl;

					// Skip neighbours which are out of bounds.
					if (dx < 0 || dz < 0 || dx >= xSize || dz >= zSize)
					{
						minNeighborHeight = (-walkableClimb - 1) ;
						if (debug)
							std::cout << "    out of bounds minNeighborHeight=" << minNeighborHeight << std::endl;
						break;
					}

					// From minus infinity to the first span.
					const rcSpan* neighborSpan = heightfield.spans[dx + dz * xSize];
					int neighborTop = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHT;

					if (debug)
						std::cout << "    top=" << top << " neighborTop=" << neighborTop
								  << " rcMin(top, neighborTop)=" << rcMin(top, neighborTop)
								  << " bot=" << bot
								  << " d=" << rcMin(top, neighborTop) - bot << std::endl;

					// Skip neighbour if the gap between the spans is too small.
					if (rcMin(top, neighborTop) - bot >= walkableHeight)
					{
						minNeighborHeight = (-walkableClimb - 1);
						if (debug)
							std::cout << "    minNeighborHeight=" << minNeighborHeight << std::endl;
						break;
					}

					int m = 0;

					// Rest of the spans.
					for (neighborSpan = heightfield.spans[dx + dz * xSize]; neighborSpan; neighborSpan = neighborSpan->next)
					{
						int neighborBot = (int)neighborSpan->smax;
						neighborTop = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHT;

						if (debug)
							std::cout << "      m=" << m
									  << " neighborBot=" << neighborBot << " neighborTop=" << neighborTop
									  << " rcMin(top, neighborTop)=" << rcMin(top, neighborTop)
									  << " rcMax(bot, neighborBot)=" << rcMax(bot, neighborBot)
									  << " d=" << rcMin(top, neighborTop) - rcMax(bot, neighborBot)
									  << std::endl;

						++m;

						// Skip neighbour if the gap between the spans is too small.
						if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) >= walkableHeight)
						{
							int accessibleNeighbourHeight = neighborBot - bot;
							minNeighborHeight = rcMin(minNeighborHeight, accessibleNeighbourHeight);

							// Find min/max accessible neighbour height.
							if (rcAbs(accessibleNeighbourHeight) <= walkableClimb)
							{
								if (neighborBot < accessibleNeighborMinHeight)
								{
									accessibleNeighborMinHeight = neighborBot;
									if (debug)
										std::cout << "      accessibleNeighborMinHeight=" << accessibleNeighborMinHeight << std::endl;
								}
								if (neighborBot > accessibleNeighborMaxHeight)
								{
									accessibleNeighborMaxHeight = neighborBot;
									if (debug)
										std::cout << "      accessibleNeighborMaxHeight=" << accessibleNeighborMaxHeight << std::endl;
								}
							}
							else if (accessibleNeighbourHeight < -walkableClimb)
							{
								if (debug)
									std::cout << "      break accessibleNeighbourHeight=" << accessibleNeighbourHeight << std::endl;
								break;
							}

						}
					}
				}

				if (debug)
					std::cout << "  end minNeighborHeight=" << minNeighborHeight
							  << " accessibleNeighborMaxHeight=" << accessibleNeighborMaxHeight
							  << " accessibleNeighborMinHeight=" << accessibleNeighborMinHeight
							  << " d=" << (accessibleNeighborMaxHeight - accessibleNeighborMinHeight)
							  << std::endl;

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				if (minNeighborHeight < -walkableClimb)
				{
					if (debug)
						std::cout << "  RC_NULL_AREA minNeighborHeight" << std::endl;
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				else if ((accessibleNeighborMaxHeight - accessibleNeighborMinHeight) > walkableClimb)
				{
					if (debug)
						std::cout << "  RC_NULL_AREA accessibleNeighborMaxHeight" << std::endl;
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);
	
	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int bot = (int)(span->smax);
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;
				if ((top - bot) < walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}
