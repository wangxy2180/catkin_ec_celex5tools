//
// Created by free on 2020/11/11.
// written by wangxy, copy from https://github.com/uzh-rpg/rpg_corner_events
#include "corner_detector/fast_detector.h"
#include "detector.h"

namespace corner_detector
{
    Fast_detector::Fast_detector()
            : Detector(), circle3{{0,  3},
                                  {1,  3},
                                  {2,  2},
                                  {3,  1},
                                  {3,  0},
                                  {3,  -1},
                                  {2,  -2},
                                  {1,  -3},
                                  {0,  -3},
                                  {-1, -3},
                                  {-2, -2},
                                  {-3, -1},
                                  {-3, 0},
                                  {-3, 1},
                                  {-2, 2},
                                  {-1, 3}},
              circle4{{0,  4},
                      {1,  4},
                      {2,  3},
                      {3,  2},
                      {4,  1},
                      {4,  0},
                      {4,  -1},
                      {3,  -2},
                      {2,  -3},
                      {1,  -4},
                      {0,  -4},
                      {-1, -4},
                      {-2, -3},
                      {-3, -2},
                      {-4, -1},
                      {-4, 0},
                      {-4, 1},
                      {-3, 2},
                      {-2, 3},
                      {-1, 4}}
    {
        SAE[0] = Eigen::MatrixXd::Zero(sensor_height, sensor_width);
        SAE[1] = Eigen::MatrixXd::Zero(sensor_height, sensor_width);
    }

    Fast_detector::~Fast_detector() throw()
    {}

    bool Fast_detector::isFeature(const celex5_msgs::event &e)
    {

//        std::chrono::time_point<std::chrono::high_resolution_clock> test_time;

//        std::chrono::time_point<std::chrono::high_resolution_clock> feature_time;
//        std::chrono::time_point<std::chrono::high_resolution_clock> feature_end_time;
//        feature_time = std::chrono::high_resolution_clock::now();

        // no polar info, so all in matrix[0]
        const int polar = 0;
        SAE[polar](e.y, e.x) = e.off_pixel_timestamp;

        const int max_scale = 1;
        const int distance_to_border = max_scale * 4;
        if (e.x < distance_to_border || e.x >= sensor_width - distance_to_border ||
            e.y < distance_to_border || e.y >= sensor_height - distance_to_border)
        {
            return false;
        }

        bool found_streak = false;
//        test_time = std::chrono::high_resolution_clock::now();
//        const auto test10_time = std::chrono::duration_cast<std::chrono::nanoseconds>(test_time - feature_time).count();
//        std::cout << "test10 time is" << test10_time << std::endl;

        for (int i = 0; i < 16; i++)
        {
            // for circle3 detect length among 3-6, for circle4 is 4-8
            for (int streak_size = 3; streak_size <= 6; streak_size++)
            {
//                test_time = std::chrono::high_resolution_clock::now();
//                const auto test101_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                        test_time - feature_time).count();
//                std::cout << "test101 time is" << test101_time << std::endl;

                /*
                 * for a streak pixel long segment, compare with its left and right neighbour
                 * index i left neighbour and index i+streak right neighbour
                 * if this two neighbour is larger than index i and i+streak,
                 * that means the condition:
                 * "higher than all other pixels on the circle" is false
                 * this segment is not target segment, exec streak++
                 * continue to judge the next streak length
                 * */

                // (i-1+16)%16 is to avoid negative
                if (SAE[polar](e.y + circle3[i][1], e.x + circle3[i][0]) <
                    SAE[polar](e.y + circle3[(i - 1 + 16) % 16][1],
                               e.x + circle3[(i - 1 + 16) % 16][0]))
                {
                    continue;
                }
//                test_time = std::chrono::high_resolution_clock::now();
//                const auto test102_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                        test_time - feature_time).count();
//                std::cout << "test102 time is" << test102_time << std::endl;

                if (SAE[polar](e.y + circle3[(i + streak_size - 1) % 16][1],
                               e.x + circle3[(i + streak_size - 1) % 16][0]) <
                    SAE[polar](e.y + circle3[(i + streak_size) % 16][1],
                               e.x + circle3[(i + streak_size) % 16][0]))
                {
                    continue;
                }
//                test_time = std::chrono::high_resolution_clock::now();
//                const auto test103_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                        test_time - feature_time).count();
//                std::cout << "test103 time is" << test103_time << std::endl;

                /*
                 * if it pass the two if statement above,
                 * we think current segment is higher than its neighbour;
                 * then we will judge other pixel in the circle
                 */

                // to find the min timestamp in target segment
                double min_t_in_seg = SAE[polar](e.y + circle3[i][1], e.x + circle3[i][0]);
                // for index between i+1 and i+2,+3,+4,+5,
                // find min timestamp in a segment of lenght streak
                for (int j = 1; j < streak_size; j++)
                {
                    const double tj = SAE[polar](e.y + circle3[(i + j) % 16][1],
                                                 e.x + circle3[(i + j) % 16][0]);
                    if (tj < min_t_in_seg)
                    {
                        min_t_in_seg = tj;
                    }
                }
//                test_time = std::chrono::high_resolution_clock::now();
//                const auto test104_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                        test_time - feature_time).count();
//                std::cout << "test104 time is" << test104_time << std::endl;

                //bool did_break = false;
                bool seg_isnot_highest = false;

                /*
                 * to find the rest point in circle
                 * if exist a pixel in other pixel in circle, its time is greater than target segment min time
                 * it can not be corner
                 */
                for (int j = streak_size; j < 16; j++)
                {
                    const double t_in_other_pixel = SAE[polar](e.y + circle3[(i + j) % 16][1],
                                                               e.x + circle3[(i + j) % 16][0]);
                    if (t_in_other_pixel >= min_t_in_seg)
                    {
                        seg_isnot_highest = true;
                        break;
                    }
                }
//                test_time = std::chrono::high_resolution_clock::now();
//                const auto test105_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                        test_time - feature_time).count();
//                std::cout << "test105 time is" << test105_time << std::endl;
                //in if above, no all of t_in_other_pixel less than min_t_in_seg
                if (!seg_isnot_highest)
                {
                    found_streak = true;
                    break;
                }
            }//end for streak
//            test_time = std::chrono::high_resolution_clock::now();
//            const auto test11_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                    test_time - feature_time).count();
//            std::cout << "test11 time is" << test11_time << std::endl;
            if (found_streak)
            { break; }
        }//end for 16
//        test_time = std::chrono::high_resolution_clock::now();
//        const auto test1_time = std::chrono::duration_cast<std::chrono::nanoseconds>(test_time - feature_time).count();
//        std::cout << "test1 time is" << test1_time << std::endl;

        /////////////////////////////////////////////////
        // now we need do same thing in circle4
        if (found_streak)
        {
            found_streak = false;
            for (int i = 0; i < 20; i++)
            {
                for (int streak_size = 4; streak_size <= 8; streak_size++)
                {
                    // to compare with segment's left and right neighbour
                    if (SAE[polar](e.y + circle4[i][1], e.x + circle4[i][0]) <
                        SAE[polar](e.y + circle4[(i - 1 + 20) % 20][1], e.x + circle4[(i - 1 + 20) % 20][0]))
                    {
                        continue;
                    }

                    if (SAE[polar](e.y + circle4[(i + streak_size - 1) % 20][1],
                                   e.x + circle4[(i + streak_size - 1) % 20][0]) <
                        SAE[polar](e.y + circle4[(i + streak_size) % 20][1], e.x + circle4[(i + streak_size) % 20][0]))
                    {
                        continue;
                    }

                    //    find min time in seg
                    double min_time_in_seg = SAE[polar](e.y + circle4[i][1], e.x + circle4[i][0]);
                    for (int j = 1; j < streak_size; j++)
                    {
                        const double t_in_segment = SAE[polar](e.y + circle4[(i + j) % 20][1],
                                                               e.x + circle4[(i + j) % 20][0]);
                        if (t_in_segment < min_time_in_seg)
                        {
                            min_time_in_seg = t_in_segment;
                        }
                    }

                    bool seg_is_highest = true;
                    for (int j = streak_size; j < 20; j++)
                    {
                        const double t_in_other_pixel = SAE[polar](e.y + circle4[(i + j) % 20][1],
                                                                   e.x + circle4[(i + j) % 20][0]);
                        if (min_time_in_seg < t_in_other_pixel)
                        {
                            seg_is_highest = false;
                            break;
                        }
                    }
                    if (seg_is_highest)
                    {
                        found_streak = true;
                        break;
                    }

                }//end for streak
                if (found_streak)
                {
                    break;
                }
            }// end for 20
        } // end  if founf streak

//        feature_end_time = std::chrono::high_resolution_clock::now();
//        const auto detect_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//                feature_end_time - feature_time).count();
//        std::cout << "one event time is" << detect_time << std::endl;

        return found_streak;
    }//end  isFeature function

} // namespace corner_detector
