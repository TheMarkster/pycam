#include "intersection.hpp"
#include <algorithm>
#include <iostream>

std::vector<intersection> find_intersections(const std::vector<std::vector<segment*>*> &paths) {
    std::vector<intersection> listIntersections;
    std::vector<segment_info> listSortedSegments;
    linked_list<segment_info> listActiveSegments;

    // Sort all segments by their x-coordinates for horizontal sweep line algorithm
    // Create list off all events (start and end points of segments)
    size_t i, id;
    segment *seg;
    segment_info si;
    id = 0;
    for (std::vector<segment*> *container : paths) {
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

    // Print hello
    // std::cout << "There are " << listSortedSegments.size() << " segments after sorting" << std::endl;

    // Scan through segments and find intersections
    size_t index = 0;
    linked_item<segment_info> *item1, *item2;
    result<vec2d> intersection_result;
    while (index < listSortedSegments.size()) {
        si = listSortedSegments[index++];

        if (!si.start) {
            // std::cout << "Processing end of segment with id: " << si.id << std::endl;
            // Remove from active segments
            item1 = listActiveSegments.head;
            while (item1) {
                if (item1->si.id == si.id) {
                    // std::cout<< "Removing segment with id: " << si.id << std::endl;
                    item1->remove();
                    break;
                }
                item1 = item1->nextItem;
            }
            
            continue;
        }
        else {
            // std::cout << "Processing start of segment with id: " << si.id << " and x " << si.x << std::endl;
            listActiveSegments.add(si);

            // Grab all segments with the same x-coordinate
            while (true) {
                if (index == listSortedSegments.size())
                    break;

                si = listSortedSegments[index];
                
                if (si.x == listActiveSegments.tail->si.x && si.start) {
                    // Add to active segments
                    // std::cout << "Adding additional segment with id: " << si.id << " and x " << si.x << std::endl;
                    listActiveSegments.add(si);
                    index++;
                }
                else {
                    break;
                }
            }
        }

        item1 = listActiveSegments.head;
        while (item1) {
            // std::cout << "Active segment with id: " << item1->si.id << " and x " << item1->si.x << std::endl;
            item1 = item1->nextItem;
        }

        // Check for intersections amongst active segments
        item1 = listActiveSegments.head;
        while (item1) {
            item2 = listActiveSegments.head;
            while (item2) {
                if (item1->si.id <= item2->si.id) {
                    // std::cout << "Skipping self-intersection for segment with id: " << item1->si.id << std::endl;
                    item2 = item2->nextItem;
                    continue; // Skip self-intersection
                }

                // std::cout << "Checking intersection between segments with ids: "
                        //   << item1->si.id << " and " << item2->si.id << std::endl;
                // Check bounding box intersection first
                if (item1->si.box.intersects(item2->si.box)) {
                    // std::cout << "Bounding boxes intersect for segments with ids: "
                    //           << item1->si.id << " and " << item2->si.id << std::endl;
                    intersection_result = item1->si.seg->intersects(*item2->si.seg);
                    // Check for actual intersection
                    if (intersection_result.success) {
                        intersection inter;
                        inter.path1 = item1->si.path;
                        inter.index1 = item1->si.index;

                        inter.path2 = item2->si.path;
                        inter.index2 = item2->si.index;

                        inter.point = intersection_result.data;

                        // Add intersection to results
                        listIntersections.push_back(inter);
                    }
                }
                item2 = item2->nextItem;
            }
            item1 = item1->nextItem;
        }
    }

    return listIntersections;
}