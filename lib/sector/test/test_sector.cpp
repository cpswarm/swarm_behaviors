#include <gtest/gtest.h>
#include "sector.h"

/**
 * @brief Test the sector helper library center function.
 */
TEST (UnitTestHelperSector, testCenter)
{
    // test center of empty sector
    sector test1 = sector(0.0, 0.0);
    EXPECT_FLOAT_EQ(0.0, test1.center());

    // test center of small sector
    sector test2 = sector(0.1, 0.2);
    EXPECT_FLOAT_EQ(0.15, test2.center());

    // test center of large sector
    sector test3 = sector(0.2, 0.1);
    EXPECT_FLOAT_EQ(0.15 + M_PI, test3.center());

    // test center of full circle
    sector test4 = sector(0.2, 0.2 + 2.0 * M_PI);
    EXPECT_FLOAT_EQ(0.2 + M_PI, test4.center());
}

/**
 * @brief Test the sector helper library contains function.
 */
TEST (UnitTestHelperSector, testContains)
{
    // test contains of empty sector
    sector test1 = sector(0.0, 0.0);
    EXPECT_TRUE(test1.contains(0.0));

    // test contains of empty sector
    sector test2 = sector(0.0, 0.0);
    EXPECT_FALSE(test2.contains(1.0));

    // test contains of small sector
    sector test3 = sector(0.1, 0.2);
    EXPECT_TRUE(test3.contains(0.15));

    // test contains of small sector
    sector test4 = sector(0.1, 0.2);
    EXPECT_FALSE(test4.contains(0.0));

    // test contains of large sector
    sector test5 = sector(0.2, 0.1);
    EXPECT_TRUE(test5.contains(0.0));

    // test contains of large sector
    sector test6 = sector(0.2, 0.1);
    EXPECT_FALSE(test6.contains(0.15));

    // test contains of full circle
    sector test7 = sector(0.2, 0.2 + 2 * M_PI);
    EXPECT_TRUE(test7.contains(0.0));

    // test contains of full circle
    sector test8 = sector(0.2, 0.2 + 2 * M_PI);
    EXPECT_TRUE(test8.contains(0.15));
}

/**
 * @brief Test the sector helper library inflate function.
 */
TEST (UnitTestHelperSector, testInflate)
{
    // test inflation of empty sector
    sector test1 = sector(0.0, 0.0);
    EXPECT_FLOAT_EQ(0.0, test1.size());
    test1.inflate(0.1);
    EXPECT_FLOAT_EQ(0.2, test1.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test1.min());
    EXPECT_FLOAT_EQ(0.1, test1.max());

    // test inflation of small sector
    sector test2 = sector(0.1, 0.2);
    test2.inflate(0.15);
    EXPECT_FLOAT_EQ(0.4, test2.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.05, test2.min());
    EXPECT_FLOAT_EQ(0.35, test2.max());

    // test inflation of large sector
    sector test3 = sector(0.2, 0.1);
    test3.inflate(0.15);
    EXPECT_FLOAT_EQ(0.15, test3.min());
    EXPECT_FLOAT_EQ(0.15, test3.max());
    EXPECT_FLOAT_EQ(0.15 + M_PI, test3.center());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test3.size());

    // test inflation of small sector with overlapping bearings
    sector test4 = sector(0.1, 0.2 + 2.0 * M_PI);
    test4.inflate(0.15);
    EXPECT_FLOAT_EQ(0.4, test4.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.05, test4.min());
    EXPECT_FLOAT_EQ(0.35, test4.max());

    // test inflation of full circle
    sector test5 = sector(0.0, 2.0 * M_PI);
    test5.inflate(0.1);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test5.size());
    EXPECT_FLOAT_EQ(0.0, test5.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test5.max_ord());
}

/**
 * @brief Test the sector helper library inverse function.
 */
TEST (UnitTestHelperSector, testInverse)
{
    // test inverse of empty sector
    sector test1 = sector(0.0, 0.0).inverse();
    EXPECT_FLOAT_EQ(2.0 * M_PI, test1.size());
    EXPECT_FLOAT_EQ(0.0, test1.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test1.max());

    // test inverse of small sector
    sector test2 = sector(0.1, 0.2).inverse();
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test2.size());
    EXPECT_FLOAT_EQ(0.2, test2.min());
    EXPECT_FLOAT_EQ(0.1, test2.max());

    // test inverse of large sector
    sector test3 = sector(0.2, 0.1).inverse();
    EXPECT_FLOAT_EQ(0.1, test3.size());
    EXPECT_FLOAT_EQ(0.1, test3.min());
    EXPECT_FLOAT_EQ(0.2, test3.max());

    // test inverse of full circle
    sector test4 = sector(0.2, 2.0 * M_PI + 0.2).inverse();
    EXPECT_NEAR(0.0, test4.size(), 0.005);
    EXPECT_FLOAT_EQ(0.2, test4.min());
    EXPECT_FLOAT_EQ(0.2, test4.max());
}

/**
 * @brief Test the sector helper library join function.
 */
TEST (UnitTestHelperSector, testJoin)
{
    //
    // test joining of two sectors where none includes the jump from 2π to 0
    //

    // overlapping sectors

    sector test1 = sector(0.0, 0.2);
    sector test1j = sector(0.1, 0.3);
    test1.join(test1j);
    EXPECT_FLOAT_EQ(0.3, test1.size());
    EXPECT_FLOAT_EQ(0.0, test1.min());
    EXPECT_FLOAT_EQ(0.3, test1.max());

    sector test1a = sector(0.0, 0.2);
    sector test1aj = sector(0.1, 0.0);
    test1a.join(test1aj);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test1a.size());
    EXPECT_FLOAT_EQ(0.0, test1a.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test1a.max());

    sector test2 = sector(0.1, 0.3);
    sector test2j = sector(0.0, 0.2);
    test2.join(test2j);
    EXPECT_FLOAT_EQ(0.3, test2.size());
    EXPECT_FLOAT_EQ(0.0, test2.min());
    EXPECT_FLOAT_EQ(0.3, test2.max());

    // one sector contains the other

    sector test3 = sector(0.1, 0.4);
    sector test3j = sector(0.2, 0.3);
    test3.join(test3j);
    EXPECT_FLOAT_EQ(0.3, test3.size());
    EXPECT_FLOAT_EQ(0.1, test3.min());
    EXPECT_FLOAT_EQ(0.4, test3.max());

    sector test3a = sector(0.0, 2.0 * M_PI);
    sector test3aj = sector(0.2, 0.3);
    test3a.join(test3aj);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test3a.size());
    EXPECT_FLOAT_EQ(0.0, test3a.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test3a.max());

    sector test4 = sector(0.2, 0.3);
    sector test4j = sector(0.1, 0.4);
    test4.join(test4j);
    EXPECT_FLOAT_EQ(0.3, test4.size());
    EXPECT_FLOAT_EQ(0.1, test4.min());
    EXPECT_FLOAT_EQ(0.4, test4.max());

    // joined sector becomes full circle

    sector test5 = sector(0.0, 1.0);
    sector test5j = sector(0.9, 2.0 * M_PI);
    test5.join(test5j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test5.size());
    EXPECT_FLOAT_EQ(0.0, test5.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test5.max());

    sector test6 = sector(2.0, 2.0 * M_PI);
    sector test6j = sector(0.0, 2.1);
    test6.join(test6j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test6.size());
    EXPECT_FLOAT_EQ(0.0, test6.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test6.max());


    //
    // test joining of two sectors where both include the jump from 2π to 0
    //

    // overlapping sectors

    sector test7 = sector(2.0 * M_PI - 0.2, 0.1);
    sector test7j = sector(2.0 * M_PI - 0.1, 0.2);
    test7.join(test7j);
    EXPECT_FLOAT_EQ(0.4, test7.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.2, test7.min());
    EXPECT_FLOAT_EQ(0.2, test7.max());

    sector test7a = sector(0.3, 0.1);
    sector test7aj = sector(0.3, 0.2);
    test7a.join(test7aj);
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test7a.size());
    EXPECT_FLOAT_EQ(0.3, test7a.min());
    EXPECT_FLOAT_EQ(0.2, test7a.max());

    sector test8 = sector(2.0 * M_PI - 0.1, 0.2);
    sector test8j = sector(2.0 * M_PI - 0.2, 0.1);
    test8.join(test8j);
    EXPECT_FLOAT_EQ(0.4, test8.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.2, test8.min());
    EXPECT_FLOAT_EQ(0.2, test8.max());

    // one sector contains the other

    sector test9 = sector(2.0 * M_PI - 0.2, 0.2);
    sector test9j = sector(2.0 * M_PI - 0.1, 0.1);
    test9.join(test9j);
    EXPECT_FLOAT_EQ(0.4, test9.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.2, test9.min());
    EXPECT_FLOAT_EQ(0.2, test9.max());

    sector test10 = sector(2.0 * M_PI - 0.1, 0.1);
    sector test10j = sector(2.0 * M_PI - 0.2, 0.2);
    test10.join(test10j);
    EXPECT_FLOAT_EQ(0.4, test10.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.2, test10.min());
    EXPECT_FLOAT_EQ(0.2, test10.max());

    // joined sector becomes full circle

    sector test11 = sector(2.0 * M_PI - 0.2, 4.0);
    sector test11j = sector(2.0, 0.2);
    test11.join(test11j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test11.size());
    EXPECT_FLOAT_EQ(0.0, test11.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test11.max());

    sector test12 = sector(2.0, 0.2);
    sector test12j = sector(2.0 * M_PI - 0.2, 4.0);
    test12.join(test12j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test12.size());
    EXPECT_FLOAT_EQ(0.0, test12.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test12.max());


    //
    // test joining of two sectors where one includes the jump from 2π to 0
    //

    // overlapping sectors

    sector test13 = sector(2.0 * M_PI - 0.1, 0.2);
    sector test13j = sector(0.1, 0.3);
    test13.join(test13j);
    EXPECT_FLOAT_EQ(0.4, test13.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test13.min());
    EXPECT_FLOAT_EQ(0.3, test13.max());

    sector test14 = sector(0.1, 0.3);
    sector test14j = sector(2.0 * M_PI - 0.1, 0.2);
    test14.join(test14j);
    EXPECT_FLOAT_EQ(0.4, test14.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test14.min());
    EXPECT_FLOAT_EQ(0.3, test14.max());

    sector test15 = sector(2.0 * M_PI - 0.2, 0.1);
    sector test15j = sector(2.0 * M_PI - 0.3, 2.0 * M_PI - 0.1);
    test15.join(test15j);
    EXPECT_FLOAT_EQ(0.4, test15.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.3, test15.min());
    EXPECT_FLOAT_EQ(0.1, test15.max());

    sector test16 = sector(2.0 * M_PI - 0.3, 2.0 * M_PI - 0.1);
    sector test16j = sector(2.0 * M_PI - 0.2, 0.1);
    test16.join(test16j);
    EXPECT_FLOAT_EQ(0.4, test16.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.3, test16.min());
    EXPECT_FLOAT_EQ(0.1, test16.max());

    // one sector contains the other

    sector test17 = sector(2.0 * M_PI - 0.1, 0.3);
    sector test17j = sector(0.1, 0.2);
    test17.join(test17j);
    EXPECT_FLOAT_EQ(0.4, test17.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test17.min());
    EXPECT_FLOAT_EQ(0.3, test17.max());

    sector test18 = sector(0.1, 0.2);
    sector test18j = sector(2.0 * M_PI - 0.1, 0.3);
    test18.join(test18j);
    EXPECT_FLOAT_EQ(0.4, test18.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test18.min());
    EXPECT_FLOAT_EQ(0.3, test18.max());

    sector test19 = sector(2.0 * M_PI - 0.3, 0.1);
    sector test19j = sector(2.0 * M_PI - 0.2, 2.0 * M_PI - 0.1);
    test19.join(test19j);
    EXPECT_FLOAT_EQ(0.4, test19.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.3, test19.min());
    EXPECT_FLOAT_EQ(0.1, test19.max());

    sector test20 = sector(2.0 * M_PI - 0.2, 2.0 * M_PI - 0.1);
    sector test20j = sector(2.0 * M_PI - 0.3, 0.1);
    test20.join(test20j);
    EXPECT_FLOAT_EQ(0.4, test20.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.3, test20.min());
    EXPECT_FLOAT_EQ(0.1, test20.max());

    // joined sector becomes full circle

    sector test21 = sector(2.0 * M_PI - 2.0, 3.0);
    sector test21j = sector(2.0, 5.0);
    test21.join(test21j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test21.size());
    EXPECT_FLOAT_EQ(0.0, test21.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test21.max());

    sector test22 = sector(2.0, 5.0);
    sector test22j = sector(2.0 * M_PI - 2.0, 3.0);
    test22.join(test22j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test22.size());
    EXPECT_FLOAT_EQ(0.0, test22.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test22.max());

    sector test23 = sector(3.0, 2.0);
    sector test23j = sector(1.0, 4.0);
    test23.join(test23j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test23.size());
    EXPECT_FLOAT_EQ(0.0, test23.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test23.max());

    sector test24 = sector(1.0, 4.0);
    sector test24j = sector(3.0, 2.0);
    test24.join(test24j);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test24.size());
    EXPECT_FLOAT_EQ(0.0, test24.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test24.max());


    //
    // test joining of two disjoint sectors
    //

    // small enclosing angle
    sector test25 = sector(0.0, 0.1);
    sector test25j = sector(0.2, 0.3);
    test25.join(test25j);
    EXPECT_FLOAT_EQ(0.3, test25.size());
    EXPECT_FLOAT_EQ(0.0, test25.min());
    EXPECT_FLOAT_EQ(0.3, test25.max());

    // large enclosing angle
    sector test26 = sector(0.0, 0.1);
    sector test26j = sector(2.0 * M_PI - 0.3, 2.0 * M_PI - 0.2);
    test26.join(test26j);
    EXPECT_FLOAT_EQ(0.4, test26.size());
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.3, test26.min());
    EXPECT_FLOAT_EQ(0.1, test26.max());

    // large enclosing angle and one includes jump from 2π to 0
    sector test27 = sector(0.2, 0.3);
    sector test27j = sector(0.4, 0.0);
    test27.join(test27j);
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.2, test27.size());
    EXPECT_FLOAT_EQ(0.2, test27.min());
    EXPECT_FLOAT_EQ(2.0 * M_PI, test27.max());
}

/**
 * @brief Test the sector helper library min/max functions.
 */
TEST (UnitTestHelperSector, testMinMax)
{
    // test min/max for empty sector
    sector test1 = sector(0.1, 0.1);
    EXPECT_FLOAT_EQ(0.1, test1.min());
    EXPECT_FLOAT_EQ(0.1, test1.min_ord());
    EXPECT_FLOAT_EQ(0.1, test1.max());
    EXPECT_FLOAT_EQ(0.1, test1.max_ord());

    // test min/max for small sector
    sector test2 = sector(0.1, 0.2);
    EXPECT_FLOAT_EQ(0.1, test2.min());
    EXPECT_FLOAT_EQ(0.1, test2.min_ord());
    EXPECT_FLOAT_EQ(0.2, test2.max());
    EXPECT_FLOAT_EQ(0.2, test2.max_ord());

    // test min/max for large sector
    sector test3 = sector(0.2, 0.1);
    EXPECT_FLOAT_EQ(0.2, test3.min());
    EXPECT_FLOAT_EQ(0.2, test3.min_ord());
    EXPECT_FLOAT_EQ(0.1, test3.max());
    EXPECT_FLOAT_EQ(2.0 * M_PI + 0.1, test3.max_ord());

    // test min/max for full circle
    sector test4 = sector(0.1, 2.0 * M_PI + 0.1);
    EXPECT_FLOAT_EQ(0.1, test4.min());
    EXPECT_FLOAT_EQ(0.1, test4.min_ord());
    EXPECT_FLOAT_EQ(0.1, test4.max());
    EXPECT_FLOAT_EQ(2.0 * M_PI + 0.1, test4.max_ord());
}

/**
 * @brief Test the sector helper library size function.
 */
TEST (UnitTestHelperSector, testSize)
{
    // test size of empty sector
    sector test1 = sector(0.0, 0.0);
    EXPECT_FLOAT_EQ(0.0, test1.size());

    // test size of small sector
    sector test2 = sector(0.1, 0.2);
    EXPECT_FLOAT_EQ(0.1, test2.size());

    // test size of large sector
    sector test3 = sector(0.2, 0.1);
    EXPECT_FLOAT_EQ(2.0 * M_PI - 0.1, test3.size());

    // test size of full circle
    sector test4 = sector(0.2, 2.0 * M_PI + 0.2);
    EXPECT_FLOAT_EQ(2.0 * M_PI, test4.size());
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
