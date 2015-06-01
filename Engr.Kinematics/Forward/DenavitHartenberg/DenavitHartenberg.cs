using Engr.Maths.Rotation;

namespace Engr.Kinematics.Forward.DenavitHartenberg
{
    public static class DenavitHartenberg
    {
        public static ParameterTable TwoRPlanar(double l1, double l2, Angle theta1, Angle theta2)
        {
            return new ParameterTable(new Parameters(l1, Angle.Zero, 0.0, theta1),new Parameters(l2, Angle.Zero, 0.0, theta2));
        }
    }
}