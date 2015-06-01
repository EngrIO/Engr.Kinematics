using System.Linq;
using Engr.Maths.Matrices;

namespace Engr.Kinematics.Forward.DenavitHartenberg
{
    public class ParameterTable
    {
        private readonly Parameters[] _frames;

        public ParameterTable(params Parameters[] frames)
        {
            _frames = frames;
        }

        public Mat4 ToTransformationMat4()
        {
            return _frames.Aggregate(Mat4.Identity, (a, b) => a * b.ToTransformationMat4());
        }
    }
}