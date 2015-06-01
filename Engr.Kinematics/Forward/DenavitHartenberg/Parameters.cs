using Engr.Maths.Matrices;
using Engr.Maths.Rotation;
using Engr.Maths.Trigonometry;

namespace Engr.Kinematics.Forward.DenavitHartenberg
{
    public class Parameters
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="a">LinkLength</param>
        /// <param name="alpha">LinkTwist</param>
        /// <param name="d">JointDistance</param>
        /// <param name="theta">JointAngle</param>
        public Parameters(double a, Angle alpha, double d, Angle theta)
        {
            LinkLength = a;
            LinkTwist = alpha;
            JointDistance = d;
            JointAngle = theta;
        }

        /// <summary>
        /// a
        /// </summary>
        public double LinkLength { get; private set; }
        /// <summary>
        /// alpha
        /// </summary>
        public Angle LinkTwist { get; private set; }
       /// <summary>
       /// d
       /// </summary>
        public double JointDistance { get; private set; }
        /// <summary>
        /// theta
        /// </summary>
        public Angle JointAngle { get; private set; }


        public Mat4 ToTransformationMat4()
        {
            return new Mat4(new[,]
            {
                {Trig.Cos(JointAngle), - Trig.Sin(JointAngle) * Trig.Cos(LinkTwist), Trig.Sin(JointAngle) * Trig.Sin(LinkTwist), LinkLength * Trig.Cos(JointAngle)},
                {Trig.Sin(JointAngle), Trig.Cos(JointAngle) * Trig.Cos(LinkTwist), - Trig.Cos(JointAngle) * Trig.Sin(LinkTwist), LinkLength * Trig.Sin(JointAngle)},
                {0, Trig.Sin(LinkTwist), Trig.Cos(LinkTwist), JointDistance},
                {0, 0, 0, 1}
            });
        }
    }
}
