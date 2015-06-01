using System;
using System.Diagnostics;
using Engr.Kinematics.Forward.DenavitHartenberg;
using Engr.Maths.Matrices;
using Engr.Maths.Rotation;
using Engr.Maths.Trigonometry;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Engr.Kinematics.Tests
{
    [TestClass]
    public class DenavitHartenbergTests
    {
        [TestMethod]
        public void TestMethod1()
        {
            var l1 = 50;
            var l2 = 60;
            var theta1 = Angle.FromDegrees(0);
            var theta2 = Angle.FromDegrees(0);

            var result = DenavitHartenberg.TwoRPlanar(l1,l2,theta1,theta2).ToTransformationMat4();

            ValidateTwoRPlanar(result, l1, l2, theta1, theta2);

        }

        private void ValidateTwoRPlanar(Mat4 result, double l1, double l2, Angle theta1, Angle theta2)
        {
            Assert.AreEqual(result[1, 1], Trig.Cos(theta1 + theta2), Constants.Delta);
            Assert.AreEqual(result[1, 2], -Trig.Sin(theta1 + theta2), Constants.Delta);
            Assert.AreEqual(result[1, 3], 0, Constants.Delta);
            Assert.AreEqual(result[1, 4], l1 * Trig.Cos(theta1) + l2 * Trig.Cos(theta1 + theta2), Constants.Delta);

            Assert.AreEqual(result[2, 1], Trig.Sin(theta1 + theta2), Constants.Delta);
            Assert.AreEqual(result[2, 2], Trig.Cos(theta1 + theta2), Constants.Delta);
            Assert.AreEqual(result[2, 3], 0, Constants.Delta);
            Assert.AreEqual(result[2, 4], l1 * Trig.Sin(theta1) + l2 * Trig.Sin(theta1 + theta2), Constants.Delta);

            Assert.AreEqual(result[3, 1], 0, Constants.Delta);
            Assert.AreEqual(result[3, 2], 0, Constants.Delta);
            Assert.AreEqual(result[3, 3], 1, Constants.Delta);
            Assert.AreEqual(result[3, 4], 0, Constants.Delta);

            Assert.AreEqual(result[4, 1], 0, Constants.Delta);
            Assert.AreEqual(result[4, 2], 0, Constants.Delta);
            Assert.AreEqual(result[4, 3], 0, Constants.Delta);
            Assert.AreEqual(result[4, 4], 1, Constants.Delta);
        }
    }
}
