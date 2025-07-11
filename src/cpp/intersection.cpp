#include "intersection.hpp"
#include "pycam.hpp"
#include <algorithm>
#include <iostream>

inline void check_intersection(const segment_info &si1, const segment_info &si2, std::vector<intersection> &results) {
    validated_intersection_result res = si1.seg->valid_intersections(*si2.seg);

    intersection inter;
    for (size_t i=0; i<res.count; i++) {
        inter.path1 = si1.path;
        inter.index1 = si1.index;
        inter.pos1 = res.data[i].pos1;

        inter.path2 = si2.path;
        inter.index2 = si2.index;
        inter.pos2 = res.data[i].pos2;

        inter.point = res.data[i].point;

        results.push_back(inter);
    }
}

std::vector<intersection> find_intersections(const std::vector<std::vector<segment*>*> &paths) {
    std::vector<intersection> listIntersections;
    std::vector<segment_info> listSortedSegments;
    linked_list<segment_info> listActiveSegments;
    std::vector<segment_info> newComers;

    // Sort all segments by their x-coordinates for horizontal sweep line algorithm
    // Create list off all events (start and end points of segments)
    size_t i, id;
    segment *seg;
    segment_info si;
    id = 0;
    for (const std::vector<segment*> *container : paths) {
        si.path = container;

        for (i=0; i<container->size(); i++) {
            // Get the segment and its bounding box
            seg = container->at(i);
            si.box = seg->get_bounding_box();
            si.index = i;
            si.seg = container->at(i);

            // Add start and end points of the segment to the sorted list
            si.id = id++;
            si.x = si.box.xmin;
            si.start = true;
            listSortedSegments.push_back(si);

            si.x = si.box.xmax;
            si.start = false;
            listSortedSegments.push_back(si);
        }
    }

    // Sort events
    std::sort(listSortedSegments.begin(), listSortedSegments.end(), [](const segment_info &a, const segment_info &b) {
        return a.x < b.x || (a.x == b.x && a.start && !b.start);
    });

    // Scan through segments and find intersections
    size_t index = 0;
    size_t index_diff;
    std::vector<linked_item<segment_info>*> itemLookup;
    itemLookup.reserve(id);
    linked_item<segment_info> *item1, *item2;
    result<vec2d> intersection_result;
    while (index < listSortedSegments.size()) {
        si = listSortedSegments[index++];

        if (!si.start) {
            // std::cout << "Processing end of segment with id: " << si.id << std::endl;
            // Remove from active segments
            itemLookup[si.id]->remove();
            continue;
        }
        else {
            // std::cout << "Processing start of segment with id: " << si.id << " and x " << si.x << std::endl;
            newComers.push_back(si);

            // Grab all segments with the same x-coordinate
            while (true) {
                if (index == listSortedSegments.size())
                    break;

                si = listSortedSegments[index];
                
                if (si.x == newComers.back().x && si.start) {
                    // Add to active segments
                    // std::cout << "Adding additional segment with id: " << si.id << " and x " << si.x << std::endl;
                    newComers.push_back(si);
                    index++;
                }
                else {
                    break;
                }
            }
        }

        // Check for intersections amongst active segments
        // item1 = listActiveSegments.head;
        for (auto &si1 : newComers) {
            item2 = listActiveSegments.head;
            while (item2) {
                if (si1.path == item2->si.path) {
                    if (si1.index > item2->si.index) {
                        index_diff = si1.index - item2->si.index;
                    }
                    else {
                        index_diff = item2->si.index - si1.index;
                    }
                    
                    if (index_diff == 1 || index_diff == si1.path->size() - 1) {
                        // Adjacent segments in the same path cannot intersect
                        item2 = item2->nextItem;
                        continue;
                    }
                }
                
                check_intersection(si1, item2->si, listIntersections);
                
                item2 = item2->nextItem;
            }
            for (auto &si2 : newComers) {
                if (si1.id <= si2.id) {
                    // No self-intersections
                    // Only one intersection per pair
                    continue;
                }
                
                if (si1.path == si2.path) {
                    if (si1.index > si2.index) {
                        index_diff = si1.index - si2.index;
                    }
                    else {
                        index_diff = si2.index - si1.index;
                    }

                    if (index_diff == 1 || index_diff == si1.path->size() - 1) {
                        // Adjacent segments in the same path cannot intersect
                        continue;
                    }
                }
                check_intersection(si1, si2, listIntersections);
            }
        }
        for (auto &si : newComers) {
            itemLookup[si.id] = listActiveSegments.add(si);
        }
        newComers.clear();
    }

    return listIntersections;
}