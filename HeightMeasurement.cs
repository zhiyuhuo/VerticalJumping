using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO;
using System.Drawing;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;

namespace VerticalJumping
{
    class FrameToStore
    {
        public double _time = 0;
        public string _state = "";
        public CameraSpacePoint[] _joints = new CameraSpacePoint[25];
    }


    class HeightMeasurement
    {
        private long _timerCount = 0;
        private Stopwatch _stopWatch = Stopwatch.StartNew();
        private Vector4 _floorPlane;
        private Body _body;
        private CameraSpacePoint[] _standingJoints = new CameraSpacePoint[25];
        CoordinateMapper _mapper;

        public double _standingKneeAngle = 0;
        public double _realTimeKneeAngle = 0;
        public double _steady = -1;
        public double _Height = 0;
        public double _HeightHead = 0;
        public double _HeightChest = 0;
        public double _HeightSpineBottom = 0;
        public string _state = "Start";

        public double _eccentricPhase = 0;
        public double _concentricPhase = 0;
        public double _timeRatio = 0;

        public double _PFLeftValgus = 0;
        public double _PFRightValgus = 0;
        public double _PFKASR = 0;

        public int m_ST = 0;
        public int m_PF = 0;
        public int m_IL = 0;

        public WriteableBitmap _colorBitmap = new WriteableBitmap(1920, 1080, 96, 96, PixelFormats.Bgr32, null);
        public WriteableBitmap _peakFlexionBitmap = new WriteableBitmap(1920, 1080, 96, 96, PixelFormats.Bgr32, null);
        public WriteableBitmap _initialLeaveBitmap = new WriteableBitmap(1920, 1080, 96, 96, PixelFormats.Bgr32, null);
        public byte[] _colorByte = new byte[1920 * 1080 * 4];

        public int _counting = 3;

        List<FrameToStore> _framesStore;
        LinkedList<WriteableBitmap> _bitMap;

        CameraSpacePoint[] JointsPeakFlexion = new CameraSpacePoint[25];
        CameraSpacePoint[] JointsInitLeave = new CameraSpacePoint[25];

        public ushort[] _depth = new ushort[512 * 424];
        public List<ushort[]> _depthSet = new List<ushort[]>();

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="body"></param>
        /// <param name="depth"></param>
        /// <param name="floorPlane"></param>
        public HeightMeasurement()
        {
            _framesStore = new List<FrameToStore>();
            _bitMap = new LinkedList<WriteableBitmap>();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="depth"></param>
        /// <param name="body"></param>
        /// <param name="floorPlane"></param>
        public void ImportData(Body body, Vector4 floorPlane, WriteableBitmap colorBitmap, ushort[] depth, CoordinateMapper mapper)
        {
            _floorPlane = floorPlane;
            _body = body;
            _colorBitmap = colorBitmap;
            for (int i = 0; i < depth.Length; i++)
            {
                _depth[i] = depth[i];
            }
            _mapper = mapper;
        }

        /// <summary>
        /// 
        /// </summary>
        public void ProcessData()
        {
            Tuple<Single, Single, Single, Single> floorClip = new Tuple<float, float, float, float>(_floorPlane.X, 
                                                                                                  _floorPlane.Y, 
                                                                                                  _floorPlane.Z, 
                                                                                                  _floorPlane.W);
            CameraSpacePoint[] skeletonPoints = FloorPlaneAdjustment(_body, floorClip);
            _realTimeKneeAngle = GetKneeAngle(skeletonPoints);

            if (_state == "Start")
            {
                _Height = 0;
                if (Math.Abs(skeletonPoints[(int)JointType.Head].X) < 0.4 
                    && Math.Abs(skeletonPoints[(int)JointType.Head].Z) < 4.0
                    && Math.Abs(skeletonPoints[(int)JointType.Head].Z) > 2.0)
                {
                    _state = "Capturing Standing Posture";
                }
            }

            else if (_state == "Capturing Standing Posture")
            {
                StoreOneFrame(skeletonPoints, _colorBitmap);

                if (Math.Abs(skeletonPoints[(int)JointType.Head].X) > 1
                    || Math.Abs(skeletonPoints[(int)JointType.Head].Z) > 4.5
                    || Math.Abs(skeletonPoints[(int)JointType.Head].Z) < 0.5)
                {
                    _state = "Start";
                }

                _standingJoints = skeletonPoints;
                _standingKneeAngle = GetKneeAngle(_standingJoints);
                _framesStore.Clear();
                _bitMap.Clear();
                _depthSet.Clear();
                _state = "Waiting for Trigger";
            }

            else if (_state == "Waiting for Trigger")
            {
                if (skeletonPoints[(int)JointType.HandLeft].Y > skeletonPoints[(int)JointType.Neck].Y
                    || skeletonPoints[(int)JointType.HandRight].Y > skeletonPoints[(int)JointType.Neck].Y)
                {   
                    //_state = "Counting Down Start";
                    _state = "Waiting for Jumping";
                }
            }

            else if (_state == "Counting Down Start")
            {
                _timerCount = _stopWatch.ElapsedMilliseconds;
                _state = "Counting Down";
            }

            else if (_state == "Counting Down")
            {
                long timerDown = _stopWatch.ElapsedMilliseconds;
                if (timerDown - _timerCount > 1000)
                {
                    _counting = 2;
                    if (timerDown - _timerCount > 2000)
                    {
                        _counting = 1;
                        if (timerDown - _timerCount > 3000)
                        {
                            _counting = 0;
                            _state = "Waiting for Jumping";
                        }
                    }
                }
            }

            else if (_state == "Waiting for Jumping")
            {
                StoreOneFrame(skeletonPoints, _colorBitmap);
                //_bitMap.AddLast(_colorBitmap.Clone());

                if (_framesStore.Count > 50)
                {
                    _framesStore.RemoveRange(0, 1);
                    //_bitMap.RemoveFirst();
                    _depthSet.RemoveRange(0, 1);
                }

                //_steady = DeviationOfJoints();
                if (Math.Abs(skeletonPoints[(int)JointType.Head].X) > 1
                    || Math.Abs(skeletonPoints[(int)JointType.Head].Z) > 4.5
                    || Math.Abs(skeletonPoints[(int)JointType.Head].Z) < 2.5)
                {
                    _state = "Start";
                }

                if (skeletonPoints[(int)JointType.AnkleLeft].Y > 0.25 && skeletonPoints[(int)JointType.AnkleRight].Y > 0.25)
                {
                    _state = "Jumping";
                }
            }

            else if (_state == "Jumping")
            {
                StoreOneFrame(skeletonPoints, _colorBitmap);
                //_bitMap.AddLast(_colorBitmap.Clone());

                double hstandHead = _standingJoints[(int)JointType.Head].Y;
                double hstandChest = _standingJoints[(int)JointType.SpineShoulder].Y;
                double hstandSpinebottom = _standingJoints[(int)JointType.SpineBase].Y;

                double dhHead = skeletonPoints[(int)JointType.Head].Y - hstandHead;
                double dhChest = skeletonPoints[(int)JointType.SpineShoulder].Y - hstandChest;
                double dhSpinebottom = skeletonPoints[(int)JointType.SpineBase].Y - hstandSpinebottom;

                if (dhHead > _HeightHead)
                {
                    _HeightHead = dhHead;
                }

                if (dhChest > _HeightChest)
                {
                    _HeightChest = dhChest;
                }

                if (dhSpinebottom > _HeightSpineBottom)
                {
                    _HeightSpineBottom = dhSpinebottom;
                }

                if (skeletonPoints[(int)JointType.AnkleLeft].Y < 0.25 && skeletonPoints[(int)JointType.AnkleRight].Y < 0.25)
                {
                    _state = "Waiting for Saving Data";
                }
            }

            else if (_state == "Waiting for Saving Data")
            {
                PostProceedData();
                _state = "Done";
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="points"></param>
        /// <returns></returns>
        public double GetBodyHeight(CameraSpacePoint[] points)
        {
            double res = 0;
            //double h = points[(int)JointType.Neck].Y
            //            + points[(int)JointType.SpineShoulder].Y
            //            + points[(int)JointType.SpineMid].Y
            //            + points[(int)JointType.SpineBase].Y;
            //h /= 4;

            double h = points[(int)JointType.Neck].Y;

            res = h;
            return res;
        }

        /// <summary>
        /// get the floor coordinate base body joint points
        /// </summary>
        /// <param name="skeleton1"></param>
        /// <param name="floorClip"></param>
        /// <returns></returns>
        private CameraSpacePoint[] FloorPlaneAdjustment(Body skeleton1, Tuple<Single, Single, Single, Single> floorClip)
        {
            //FOR ERIK: what does xd stand for? first vector
            var vector1 = new Single[3];
            vector1[0] = floorClip.Item1;
            vector1[1] = floorClip.Item2;
            vector1[2] = floorClip.Item3;

            //FOR ERIK: what does x2 stand for? second vector that defines a coordinate system with respect to the ground plane
            var vector2 = new Single[3];
            vector2[0] = 0;
            vector2[1] = vector1[0];
            vector2[2] = vector1[1];

            //FOR ERIK: what does vmag stand for? vertical magnitude
            var verticalMagnitude = Math.Sqrt((vector2[1] * vector2[1]) + (vector2[2] * vector2[2]));
            vector2[0] = Convert.ToSingle(vector2[0] / verticalMagnitude);
            vector2[1] = Convert.ToSingle(vector2[1] / verticalMagnitude);
            vector2[2] = Convert.ToSingle(vector2[2] / verticalMagnitude);

            //FOR ERIK: what does x3 stand for? third vector
            var vector3 = new Single[3];
            vector3[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
            vector3[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
            vector3[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
            if (vector3[0] < 0)
            {
                //Need it to go positive x...
                vector3[0] = -vector3[0];
                vector3[1] = -vector3[1];
                vector3[2] = -vector3[2];
            }

            //FOR ERIK: what does rData stand for? rotation matrix data
            var rotationMatData = new Single[9];
            //Right
            rotationMatData[0] = vector3[0];
            rotationMatData[1] = vector3[1];
            rotationMatData[2] = vector3[2];
            //Up, since Y is up
            rotationMatData[3] = vector1[0];
            rotationMatData[4] = vector1[1];
            rotationMatData[5] = vector1[2];
            //Forward
            rotationMatData[6] = vector2[0];
            rotationMatData[7] = vector2[1];
            rotationMatData[8] = vector2[2];

            //FOR ERIK: what does nj stand for? number of joints
            var jointCount = Enum.GetNames(typeof(JointType)).Length;

            //FOR ERIK: what does ts stand for? temp skeleton point array
            var tempSkelPointArray = new CameraSpacePoint[jointCount];

            for (var jointIndex = 0; jointIndex < jointCount; jointIndex++)
            {
                tempSkelPointArray[jointIndex] = AdjustJoint(skeleton1.Joints[(JointType)jointIndex], rotationMatData, floorClip.Item4);
            }
            //Console.WriteLine("floorclip[4]={0}", floorClip.Item4);

            //Console.WriteLine("rdata = {0} {1} {2} {3} {4} {5} {6} {7} {8}", rData[0], rData[1], rData[2], rData[3], rData[4], rData[5], rData[6], rData[7], rData[8]);

            return tempSkelPointArray;
        }

        private CameraSpacePoint[] FloorPlaneAdjustment(CameraSpacePoint[] ptSet, Tuple<Single, Single, Single, Single> floorClip)
        {
            //FOR ERIK: what does xd stand for? first vector
            var vector1 = new Single[3];
            vector1[0] = floorClip.Item1;
            vector1[1] = floorClip.Item2;
            vector1[2] = floorClip.Item3;

            //FOR ERIK: what does x2 stand for? second vector that defines a coordinate system with respect to the ground plane
            var vector2 = new Single[3];
            vector2[0] = 0;
            vector2[1] = vector1[0];
            vector2[2] = vector1[1];

            //FOR ERIK: what does vmag stand for? vertical magnitude
            var verticalMagnitude = Math.Sqrt((vector2[1] * vector2[1]) + (vector2[2] * vector2[2]));
            vector2[0] = Convert.ToSingle(vector2[0] / verticalMagnitude);
            vector2[1] = Convert.ToSingle(vector2[1] / verticalMagnitude);
            vector2[2] = Convert.ToSingle(vector2[2] / verticalMagnitude);

            //FOR ERIK: what does x3 stand for? third vector
            var vector3 = new Single[3];
            vector3[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
            vector3[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
            vector3[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
            if (vector3[0] < 0)
            {
                //Need it to go positive x...
                vector3[0] = -vector3[0];
                vector3[1] = -vector3[1];
                vector3[2] = -vector3[2];
            }

            //FOR ERIK: what does rData stand for? rotation matrix data
            var rotationMatData = new Single[9];
            //Right
            rotationMatData[0] = vector3[0];
            rotationMatData[1] = vector3[1];
            rotationMatData[2] = vector3[2];
            //Up, since Y is up
            rotationMatData[3] = vector1[0];
            rotationMatData[4] = vector1[1];
            rotationMatData[5] = vector1[2];
            //Forward
            rotationMatData[6] = vector2[0];
            rotationMatData[7] = vector2[1];
            rotationMatData[8] = vector2[2];

            //FOR ERIK: what does nj stand for? number of joints
            var jointCount = Enum.GetNames(typeof(JointType)).Length;

            //FOR ERIK: what does ts stand for? temp skeleton point array
            var tempSkelPointArray = new CameraSpacePoint[ptSet.Length];

            for (int i = 0; i < ptSet.Length; i++ )
            {
                Joint jt = new Joint();
                jt.Position.X = ptSet[i].X;
                jt.Position.Y = ptSet[i].Y;
                jt.Position.Z = ptSet[i].Z;
                tempSkelPointArray[i] = AdjustJoint(jt, rotationMatData, floorClip.Item4);
            }
            //Console.WriteLine("floorclip[4]={0}", floorClip.Item4);

            //Console.WriteLine("rdata = {0} {1} {2} {3} {4} {5} {6} {7} {8}", rData[0], rData[1], rData[2], rData[3], rData[4], rData[5], rData[6], rData[7], rData[8]);

            return tempSkelPointArray;
        }

        /// <summary>
        /// assist the FloorPlaneAdjustment function
        /// </summary>
        /// <param name="j"></param>
        /// <param name="rData"></param>
        /// <param name="fY"></param>
        /// <returns></returns>
        private CameraSpacePoint AdjustJoint(Joint j, Single[] rData, Single fY)
        {
            var x = rData[0] * j.Position.X + rData[1] * j.Position.Y + rData[2] * j.Position.Z;
            var y = rData[3] * j.Position.X + rData[4] * j.Position.Y + rData[5] * j.Position.Z + fY;
            var z = rData[6] * j.Position.X + rData[7] * j.Position.Y + rData[8] * j.Position.Z;

            //FOR ERIK: what does tj stand for? temporary joint or new joint
            var tempJoint = new CameraSpacePoint
            {
                X = x,
                Y = y,
                Z = z
            };

            return tempJoint;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="joints"></param>
        private void StoreOneFrame(CameraSpacePoint[] joints, WriteableBitmap colorBitmap)
        {
            FrameToStore f = new FrameToStore();
            TimeSpan t = _stopWatch.Elapsed;
            f._time = t.TotalMilliseconds;
            f._joints = joints;
            f._state = this._state;
            _framesStore.Add(f);
            ushort[] dt = new ushort[_depth.Length];
            for (int i = 0; i < _depth.Length; i++)
            {
                dt[i] = _depth[i];
            }
            _depthSet.Add(dt);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="joint1"></param>
        /// <param name="joint2"></param>
        /// <returns></returns>
        private double DifferentBetweenTwoFrames(CameraSpacePoint[] joint1, CameraSpacePoint[] joint2)
        {
            double res = 0;

            double d = Math.Abs(joint2[(int)JointType.Head].Z - joint1[(int)JointType.Head].Z)
                + Math.Abs(joint2[(int)JointType.SpineBase].Z - joint1[(int)JointType.SpineBase].Z)
                + Math.Abs(joint2[(int)JointType.KneeLeft].Z - joint1[(int)JointType.KneeLeft].Z)
                 + Math.Abs(joint2[(int)JointType.KneeRight].Z - joint1[(int)JointType.KneeRight].Z);
            d /= 4;

            res = d;
            return res;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private double DeviationOfFourBodyJoints()
        {
            double res = -1;

            int LENGTH = 60;

            double headMean = 0;
            double neckMean = 0;
            double spinemidMean = 0;
            double spinebaseMean = 0;

            double headDev = 0;
            double neckDev = 0;
            double spinemidDev = 0;
            double spinebaseDev = 0;

            if (_framesStore.Count > LENGTH)
            {
                for (int i = _framesStore.Count - LENGTH; i < _framesStore.Count; i++)
                {
                    headMean += _framesStore[i]._joints[(int)JointType.Head].Y;
                    neckMean += _framesStore[i]._joints[(int)JointType.Neck].Y;
                    spinemidMean += _framesStore[i]._joints[(int)JointType.SpineMid].Y;
                    spinebaseMean += _framesStore[i]._joints[(int)JointType.SpineBase].Y;
                }

                headMean /= LENGTH;
                neckMean /= LENGTH;
                spinemidMean /= LENGTH;
                spinebaseMean /= LENGTH;

                for (int i = _framesStore.Count - LENGTH; i < _framesStore.Count; i++)
                {
                    headDev += Math.Pow(_framesStore[i]._joints[(int)JointType.Head].Y - headMean, 2);
                    neckDev += Math.Pow(_framesStore[i]._joints[(int)JointType.Neck].Y - neckMean, 2);
                    spinemidDev +=Math.Pow( _framesStore[i]._joints[(int)JointType.SpineMid].Y - spinemidMean, 2);
                    spinebaseDev += Math.Pow(_framesStore[i]._joints[(int)JointType.SpineBase].Y - spinebaseMean, 2);
                }

                headDev /= LENGTH;
                headDev = Math.Sqrt(headDev);
                neckDev /= LENGTH;
                neckDev = Math.Sqrt(neckDev);
                spinemidDev /= LENGTH;
                spinemidDev = Math.Sqrt(spinemidDev);
                spinebaseDev /= LENGTH;
                spinebaseDev = Math.Sqrt(spinebaseDev);

                res = headDev + neckDev + spinemidDev + spinebaseDev;
            }

            return res;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private double DeviationOfKneeAnglesJoints(CameraSpacePoint[] joints)
        {
            double LUpX = joints[(int)JointType.KneeLeft].X - joints[(int)JointType.HipLeft].X;
            double LUpY = joints[(int)JointType.KneeLeft].Y - joints[(int)JointType.HipLeft].Y;
            double LUpZ = joints[(int)JointType.KneeLeft].Z - joints[(int)JointType.HipLeft].Z;

            double RUpX = joints[(int)JointType.KneeRight].X - joints[(int)JointType.HipRight].X;
            double RUpY = joints[(int)JointType.KneeRight].Y - joints[(int)JointType.HipRight].Y;
            double RUpZ = joints[(int)JointType.KneeRight].Z - joints[(int)JointType.HipRight].Z;

            double LDownX = joints[(int)JointType.AnkleLeft].X - joints[(int)JointType.KneeLeft].X;
            double LDownY = joints[(int)JointType.AnkleLeft].Y - joints[(int)JointType.KneeLeft].Y;
            double LDownZ = joints[(int)JointType.AnkleLeft].Z - joints[(int)JointType.KneeLeft].Z;

            double RDownX = joints[(int)JointType.AnkleRight].X - joints[(int)JointType.KneeRight].X;
            double RDownY = joints[(int)JointType.AnkleRight].Y - joints[(int)JointType.KneeRight].Y;
            double RDownZ = joints[(int)JointType.AnkleRight].Z - joints[(int)JointType.KneeRight].Z;

            double costhetaleft = (LUpX * LDownX + LUpY * LDownY + LUpZ * LDownZ)
                / (Math.Sqrt(LUpX * LUpX + LUpY * LUpY + LUpZ * LUpZ) * Math.Sqrt(LDownX * LDownX + LDownY * LDownY + LDownZ * LDownZ));

            double costhetaright = (RUpX * RDownX + RUpY * RDownY + RUpZ * RDownZ)
                / (Math.Sqrt(RUpX * RUpX + RUpY * RUpY + RUpZ * RUpZ) * Math.Sqrt(RDownX * RDownX + RDownY * RDownY + RDownZ * RDownZ));

            double angleleft = Math.Acos(costhetaleft);
            double angleright = Math.Acos(costhetaright);
            double res = (angleleft + angleright) / 2;

            return res;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private int PostProceedData()
        {
            int res = 0;

            int IL = 0;
            List<double[]> r0 = new List<double[]>();
            for (int i = 0; i < _framesStore.Count; i++)
            {
                double[] r = GetGap(_depthSet[i]);
                double il = 0;
                for (int j = 0; j < 10; j++)
                {
                    if (r[j] == 0)
                    {
                        il++;
                    }
                }
                if (il >= 5)
                {
                    IL = i;
                    break;
                }
                r0.Add(r);
            }

            List<double> kneeAngleList = new List<double>();
            for (int i = 0; i < _framesStore.Count; i++)
            {
                kneeAngleList.Add(GetKneeAngle(_framesStore[i]._joints));
            }

            // get jumping up key frame to separate the peak flexion point
            int keyFrame = 0;
            for (int i = 0; i < _framesStore.Count; i++)
            {
                if (_framesStore[i]._state == "Waiting for Jumping" && _framesStore[i + 1]._state == "Jumping")
                {
                    keyFrame = i;
                }
            }

            // get the peak flexion point frame
            double kneeAnglePF = 0;
            int keyFramePF = 0;
            for (int i = 0; i < keyFrame; i++)
            {
                double kneeAngle = DeviationOfKneeAnglesJoints(_framesStore[i]._joints);
                if (kneeAngle > kneeAnglePF)
                {
                    kneeAnglePF = kneeAngle;
                    keyFramePF = i;
                }
            }

            //int keyFramePF = 0;
            //double bodyHeight = GetSkeletonHeight(_standingJoints);
            //for (int i = 0; i < keyFrame; i++)
            //{
            //    double h = GetSkeletonHeight(_framesStore[i]._joints);
            //    if (h < bodyHeight)
            //    {
            //        bodyHeight = h;
            //        keyFramePF = i;
            //    }
            //}

            // get starting squatting frame
            double standingstillKneeHeight = (_standingJoints[(int)JointType.KneeLeft].Y + _standingJoints[(int)JointType.KneeRight].Y) / 2;
            int keyFrameStart = 0;
            for (int i = keyFramePF; i >= 0; i--)
            {
                double kneeHeights = (_framesStore[i]._joints[(int)JointType.KneeLeft].Y + _framesStore[i]._joints[(int)JointType.KneeRight].Y) / 2;
                double kneeAngle = GetKneeAngle(_framesStore[i]._joints);
                if (kneeAngle < _standingKneeAngle + 5)
                {
                    keyFrameStart = i;
                    break;
                }
                //double d = DifferentBetweenTwoFrames(_framesStore[i]._joints, _framesStore[i + 1]._joints);
                //if (d < 0.01 && Math.Abs(kneeHeights - standingstillKneeHeight) < 0.05)
                //{
                //    keyFrameStart = i;
                //    break;
                //}
            }

            m_ST = keyFrameStart;

            // get initial leaving frame
            int keyFrameInitialLeave = 0;
            for (int i = keyFramePF + 1; i < _framesStore.Count - 3; i++)
            {
                double PFjheight = (_framesStore[keyFramePF]._joints[(int)JointType.AnkleLeft].Y + _framesStore[keyFramePF]._joints[(int)JointType.AnkleRight].Y) / 2;
                double jheight = (_framesStore[i]._joints[(int)JointType.AnkleLeft].Y + _framesStore[i]._joints[(int)JointType.AnkleRight].Y) / 2;

                if (jheight - PFjheight > 0.08)
                {
                    keyFrameInitialLeave = i;
                    break;
                }
            }

            m_PF = keyFramePF;
            //m_IL = keyFrameInitialLeave;
            m_IL = IL;

            _eccentricPhase = _framesStore[keyFramePF]._time - _framesStore[keyFrameStart]._time;
            _concentricPhase = _framesStore[keyFrameInitialLeave]._time - _framesStore[keyFramePF]._time;
            _timeRatio = _eccentricPhase / _concentricPhase;

            JointsPeakFlexion = _framesStore[keyFramePF]._joints;
            JointsInitLeave = _framesStore[keyFrameInitialLeave]._joints;

            //_peakFlexionBitmap = _bitMap.ElementAt(keyFramePF);
            //_initialLeaveBitmap = _bitMap.ElementAt(keyFrameInitialLeave);

            // left valgus
            CameraSpacePoint kl = _framesStore[keyFramePF]._joints[(int)JointType.KneeLeft];
            CameraSpacePoint kr = _framesStore[keyFramePF]._joints[(int)JointType.KneeRight];
            CameraSpacePoint al = _framesStore[keyFramePF]._joints[(int)JointType.AnkleLeft];
            CameraSpacePoint ar = _framesStore[keyFramePF]._joints[(int)JointType.AnkleRight];

            _PFLeftValgus = Math.Atan2(kl.X - al.X, kl.Y - al.Y) * 180 / Math.PI;
            _PFRightValgus = Math.Atan2(ar.X - kr.X, kr.Y - ar.Y) * 180 / Math.PI;
            _PFKASR = Math.Abs(kl.X - kr.X) / Math.Abs(al.X - ar.X);

            return res;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public double GetSkeletonHeight(CameraSpacePoint[] joints)
        {
            double res = 0;

            res = joints[(int)JointType.KneeLeft].Y + joints[(int)JointType.KneeRight].Y + joints[(int)JointType.Head].Y;
            res /= 3;

            return res;
        }

        /// <summary>
        /// 
        /// </summary>
        public void SaveFrames()
        {
            //save rgb imgs
            LinkedList<WriteableBitmap>.Enumerator imageListEnum = _bitMap.GetEnumerator();
            imageListEnum.MoveNext();
            for (Int32 k = 0; k < Math.Min(_bitMap.Count, 50); k++)
            {
                Bitmap original;
                MemoryStream outStream = new MemoryStream();
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create((BitmapSource)(imageListEnum.Current)));
                enc.Save(outStream);
                original = new System.Drawing.Bitmap(outStream);
                Bitmap resized = new Bitmap(original, new System.Drawing.Size(original.Width / 5, original.Height / 5));
                resized.Save("C:/Users/Public/Jumping Data/img temp/" + k.ToString() + ".png");
                outStream.Close();
                original.Dispose();
                resized.Dispose();
                imageListEnum.MoveNext();
            }

            String strbody = "";
            for (int i = 0; i < _framesStore.Count; i++)
            {
                String substr = "";
                FrameToStore f = _framesStore[i];
                //substr += f._time.ToString() + " " + "0" + " " + "0" + "\n";
                for (int j = 0; j < Enum.GetNames(typeof(JointType)).Length; j++)
                {
                    substr += f._joints[j].X.ToString() + " " + f._joints[j].Y.ToString() + " " + f._joints[j].Z.ToString() + "\n";
                }
                strbody += substr;
            }
            _framesStore.Clear();
            _depthSet.Clear();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public double GetKneeAngle(CameraSpacePoint[] joints)
        {
            double res = 0;

            double lkx = joints[(int)JointType.KneeLeft].X;
            double lky = joints[(int)JointType.KneeLeft].Y;
            double lkz = joints[(int)JointType.KneeLeft].Z;

            double lhx = joints[(int)JointType.HipLeft].X;
            double lhy = joints[(int)JointType.HipLeft].Y;
            double lhz = joints[(int)JointType.HipLeft].Z;

            double lax = joints[(int)JointType.AnkleLeft].X;
            double lay = joints[(int)JointType.AnkleLeft].Y;
            double laz = joints[(int)JointType.AnkleLeft].Z;

            double rkx = joints[(int)JointType.KneeRight].X;
            double rky = joints[(int)JointType.KneeRight].Y;
            double rkz = joints[(int)JointType.KneeRight].Z;

            double rhx = joints[(int)JointType.HipRight].X;
            double rhy = joints[(int)JointType.HipRight].Y;
            double rhz = joints[(int)JointType.HipRight].Z;

            double rax = joints[(int)JointType.AnkleRight].X;
            double ray = joints[(int)JointType.AnkleRight].Y;
            double raz = joints[(int)JointType.AnkleRight].Z;

            double lux = lkx - lhx;
            double luy = lky - lhy;
            double luz = lkz - lhz;

            double ldx = lkx - lax;
            double ldy = lky - lay;
            double ldz = lkz - laz;

            double rux = rkx - rhx;
            double ruy = rky - rhy;
            double ruz = rkz - rhz;

            double rdx = rkx - rax;
            double rdy = rky - ray;
            double rdz = rkz - raz;

            double lcos = (lux * ldx + luy * ldy + luz * ldz) / (Math.Sqrt(lux * lux + luy * luy + luz * luz) * Math.Sqrt(ldx * ldx + ldy * ldy + ldz * ldz));
            double rcos = (rux * rdx + ruy * rdy + ruz * rdz) / (Math.Sqrt(rux * rux + ruy * ruy + ruz * ruz) * Math.Sqrt(rdx * rdx + rdy * rdy + rdz * rdz));

            double langle = (Math.PI - Math.Acos(lcos)) * 180 / Math.PI;
            double rangle = (Math.PI - Math.Acos(rcos)) * 180 / Math.PI;

            res = (langle + rangle) / 2;
            return res;
        }

        public double[] GetGap(ushort[] depth)
        {
            double res = 0;
            CameraSpacePoint[] pt = new CameraSpacePoint[101*212];
            DepthSpacePoint[] dd = new DepthSpacePoint[101*212];
            ushort[] dp = new ushort[101*212];
            int t = 0;
            for (int u = 512 / 2 - 50; u <= 512 / 2 + 50; u++ )
            {
                for (int v = 424-1; v >= 424/2+1; v--)
                {
                    dd[t].X = u;
                    dd[t].Y = v;
                    dp[t] = depth[u + v * 512];
                    t++;
                }
            }

            _mapper.MapDepthPointsToCameraSpace(dd, dp, pt);
            Tuple<Single, Single, Single, Single> floorClip = new Tuple<float, float, float, float>(_floorPlane.X,
                                                                                                  _floorPlane.Y,
                                                                                                  _floorPlane.Z,
                                                                                                  _floorPlane.W);
            CameraSpacePoint[] floorPts = FloorPlaneAdjustment(pt, floorClip);
            double[] h = new double[10];
            for (int i = 0; i < floorPts.Length; i++)
            {
                if (floorPts[i].X > -0.5 && floorPts[i].X < 0.5 && floorPts[i].Z > 3 && floorPts[i].Z < 4)
                {
                    int c = (int)(floorPts[i].Y / 0.01);
                    if (c >= 0 && c < 10)
                    {
                        h[c]++;
                    }
                }
            }
            for (int i = 0; i < 10; i++)
            {
                if (h[i] == 0)
                {
                    res++;
                }
            }
                
            return h;
        }
    }
}
