#include <gtest/gtest.h>

class graphSearchTest: public ::testing::Test {
public:
   graphSearchTest() {
       // initialization code here
   }

   void SetUp() {
       // code here will execute just before the test ensues
   }

   void TearDown() {
       // code here will be called just after the test completes
       // ok to through exceptions from here if need be
   }

   ~graphSearchTest()  {
       // cleanup any pending stuff, but no exceptions allowed
   }

   // put in any custom data members that you need
};

TEST_F (graphSearchTest, UnitTest1) {
  EXPECT_EQ(18.0, 18.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
