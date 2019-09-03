#include <gtest/gtest.h>
#include "angle.h"

/**
 * @brief Test the angle helper library constructors.
 */
TEST (UnitTestHelperAngle, testConstruction)
{
    // test default constructor
    angle test1 = angle();
    EXPECT_FLOAT_EQ(0.0, test1.rad());

    // test the set constructor
    angle test2 = angle(-1.0);
    EXPECT_FLOAT_EQ(-1.0, test2.rad());

    // test the set constructor
    angle test3 = angle(1.0);
    EXPECT_FLOAT_EQ(1.0, test3.rad());

    // test the set constructor for degree
    angle test4 = angle(1.0, false);
    EXPECT_FLOAT_EQ(1.0, test4.deg());
}

/**
 * @brief Test the angle helper library set function.
 */
TEST (UnitTestHelperAngle, testSetting)
{
    // test set function
    angle test1 = angle();
    test1.set(1.0);
    EXPECT_FLOAT_EQ(1.0, test1.rad());

    // test set function
    angle test2 = angle();
    test2.set(-1.0);
    EXPECT_FLOAT_EQ(-1.0, test2.rad());

    // test set function for degree
    angle test3 = angle();
    test3.set_deg(-1.0);
    EXPECT_FLOAT_EQ(-1.0, test3.deg());
}

/**
 * @brief Test the angle helper library get functions.
 */
TEST (UnitTestHelperAngle, testGetting)
{
    // test getting radian
    angle test1 = angle(1.0);
    EXPECT_FLOAT_EQ(1.0, test1.rad());

    // test getting radian
    angle test2 = angle(-1.0);
    EXPECT_FLOAT_EQ(-1.0, test2.rad());

    // test getting positive radian
    angle test3 = angle(1.0);
    EXPECT_FLOAT_EQ(1.0, test3.rad_pos());

    // test getting positive radian
    angle test4 = angle(-1.0);
    EXPECT_FLOAT_EQ(2.0 * M_PI - 1.0, test4.rad_pos());

    // test getting degree
    angle test5 = angle(1.0);
    EXPECT_FLOAT_EQ(1.0 / M_PI * 180.0, test5.deg());

    // test getting degree
    angle test6 = angle(-1.0);
    EXPECT_FLOAT_EQ(-1.0 / M_PI * 180.0, test6.deg());
}

/**
 * @brief Test the angle helper library normalization.
 */
TEST (UnitTestHelperAngle, testNormalization)
{
    // test the normalization for radian
    angle test1 = angle(3.5 * M_PI);
    EXPECT_FLOAT_EQ(-M_PI / 2.0, test1.rad());

    // test the normalization for radian
    angle test2 = angle(-3.5 * M_PI);
    EXPECT_FLOAT_EQ(M_PI / 2.0, test2.rad());

    // test the normalization for degree
    angle test3 = angle(3.5 * M_PI);
    EXPECT_FLOAT_EQ(-90.0, test3.deg());

    // test the normalization for degree
    angle test4 = angle(-3.5 * M_PI);
    EXPECT_FLOAT_EQ(90.0, test4.deg());
}

/**
 * @brief Test the angle helper library operators.
 */
TEST (UnitTestHelperAngle, testOperators)
{
    // test the + operator for integer
    angle test1 = angle(3.5 * M_PI) + 1;
    EXPECT_FLOAT_EQ(-M_PI / 2.0 + 1.0, test1.rad());

    // test the + operator for double
    angle test2 = angle(3.5 * M_PI) + M_PI;
    EXPECT_FLOAT_EQ(M_PI / 2.0, test2.rad());

    // test the + operator for two angles
    angle test3 = angle(3.5 * M_PI) + angle(M_PI);
    EXPECT_FLOAT_EQ(M_PI / 2.0, test3.rad());

    // test the += operator
    angle test4 = angle(3.5 * M_PI);
    test4 += angle(M_PI);
    EXPECT_FLOAT_EQ(M_PI / 2.0, test4.rad());

    // test the - operator for two angles
    angle test5 = angle(3.5 * M_PI) - angle(M_PI);
    EXPECT_FLOAT_EQ(M_PI / 2.0, test5.rad());

    // test the -= operator
    angle test6 = angle(3.5 * M_PI);
    test6 -= angle(M_PI);
    EXPECT_FLOAT_EQ(M_PI / 2.0, test6.rad());

    // test the * operator for double
    angle test7 = angle(3.5 * M_PI) * 1.5;
    EXPECT_FLOAT_EQ(1.0 / 4.0 * M_PI, test7.rad());

    // test the == operator
    EXPECT_TRUE(angle(3.0 * M_PI) == angle(M_PI));

    // test the == operator
    EXPECT_FALSE(angle(2.0 * M_PI) == angle(M_PI));

    // test the < operator
    EXPECT_TRUE(angle(M_PI / 2.0) < angle(M_PI));

    // test the < operator
    EXPECT_FALSE(angle(M_PI) < angle(M_PI / 2.0));

    // test the > operator
    EXPECT_FALSE(angle(M_PI / 2.0) > angle(M_PI));

    // test the > operator
    EXPECT_TRUE(angle(M_PI) > angle(M_PI / 2.0));

    // test the <= operator
    EXPECT_TRUE(angle(M_PI / 2.0) <= angle(M_PI));

    // test the <= operator
    EXPECT_TRUE(angle(M_PI) <= angle(M_PI));

    // test the <= operator
    EXPECT_FALSE(angle(M_PI) <= angle(M_PI / 2.0));

    // test the >= operator
    EXPECT_FALSE(angle(M_PI / 2.0) >= angle(M_PI));

    // test the >= operator
    EXPECT_TRUE(angle(M_PI) >= angle(M_PI));

    // test the >= operator
    EXPECT_TRUE(angle(M_PI) >= angle(M_PI / 2.0));
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
