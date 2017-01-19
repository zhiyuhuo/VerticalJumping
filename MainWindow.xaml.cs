//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace VerticalJumping
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Text;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// counter
        /// </summary>
        private int counter = 0;

        /// <summary>
        /// 
        /// </summary>
        private HeightMeasurement HeightMeasure;

        /// <summary>
        /// 
        /// </summary>
        private bool IfGroundCalib = false;

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        private Vector4 FloorPlane = new Vector4();

        private ushort[] depth = new ushort[512 * 424];

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroupInitLeave;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSourceInitLeave;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroupPeakFlexion;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSourcePeakFlexion;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// 
        /// </summary>
        private FrameDescription colorFrameDescription = null;

        /// <summary>
        /// 
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Reader for coloe frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            // get size of joint space
            this.displayWidth = colorFrameDescription.Width;
            this.displayHeight = colorFrameDescription.Height;

            BuildSkeletonModel();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();
            depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = null;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Create the drawing group we'll use for drawing
            this.drawingGroupInitLeave = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSourceInitLeave = new DrawingImage(this.drawingGroupInitLeave);

            // Create the drawing group we'll use for drawing
            this.drawingGroupPeakFlexion = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSourcePeakFlexion = new DrawingImage(this.drawingGroupPeakFlexion);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // inital height measure class
            HeightMeasure = new HeightMeasurement();

            this.WindowState = WindowState.Normal;
            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// build the skeleton model
        /// </summary>
        private void BuildSkeletonModel()
        {
            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.YellowGreen, 8));
            this.bodyColors.Add(new Pen(Brushes.Orange, 8));
            this.bodyColors.Add(new Pen(Brushes.Green, 8));
            this.bodyColors.Add(new Pen(Brushes.LightBlue, 8));
            this.bodyColors.Add(new Pen(Brushes.Red, 8));
            this.bodyColors.Add(new Pen(Brushes.Firebrick, 8));
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSquattingLowest
        {
            get
            {
                return this.imageSourcePeakFlexion;
            }
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageInitialLeave
        {
            get
            {
                return this.imageSourceInitLeave;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;
            }

            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;
            }

            if (this.depthFrameReader != null)
            {
                this.depthFrameReader.FrameArrived += this.Reader_DepthFrameArrived;
            }

            this.WindowState = WindowState.Maximized;
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            counter++;
            DrawKeyFramesPicture();
            using (DrawingContext dc = this.drawingGroup.Open())
            {
                this.DrawColorImage(dc);
                this.DrawStandLine(dc);

                using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
                {
                    if (bodyFrame != null)
                    {
                        if (this.bodies == null)
                        {
                            this.bodies = new Body[bodyFrame.BodyCount];
                        }

                        if (IfGroundCalib == false)
                        {
                            FloorPlane = bodyFrame.FloorClipPlane;
                        }
                    

                        // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                        // As long as those body objects are not disposed and not set to null in the array,
                        // those body objects will be re-used.
                        bodyFrame.GetAndRefreshBodyData(this.bodies);
                        dataReceived = true;
                    }
                }

                if (dataReceived)
                {

                        // Draw a transparent background to set the render size
                        //dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                        int penIndex = 0;

                        foreach (Body body in this.bodies)
                        {
                            Pen drawPen = this.bodyColors[penIndex++];
                            if (body.IsTracked && body.Lean.X < 0.75 && body.Lean.X > -0.75 && body.Lean.Y < 4.5)
                            {
                                this.DrawClippedEdges(body, dc);

                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                                // convert the joint points to depth (display) space
                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                                foreach (JointType jointType in joints.Keys)
                                {
                                    // sometimes the depth(Z) of an inferred joint may show as negative
                                    // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                    CameraSpacePoint position = joints[jointType].Position;
                                    if (position.Z < 0)
                                    {
                                        position.Z = InferredZPositionClamp;
                                    }

                                    ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);
                                    jointPoints[jointType] = new Point(colorSpacePoint.X, colorSpacePoint.Y);
                                
                                }


                                this.DrawBody(joints, jointPoints, dc, drawPen);
                                HeightMeasure.ImportData(body, FloorPlane, colorBitmap, depth, coordinateMapper);
                                if (HeightMeasure._state == "Counting Down" || HeightMeasure._state == "Waiting for Jumping")
                                {
                                    String str = HeightMeasure._counting.ToString();
                                    if (str == "0")
                                        str = "Go";
                                    DrawTextOnScreen(str, 100, 100, dc);
                                }
                                HeightMeasure.ProcessData();
                                ControlRectByStillness(HeightMeasure._steady);
                                UpdateJumpMeasureState();

                                break;
                            }
                        }

                        // prevent drawing outside of our render area
                        this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                
                }
                else
                {

                }
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private unsafe void Reader_DepthFrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        ushort* frameData = (ushort*)(depthBuffer.UnderlyingBuffer);
                        for (int i = 0; i < 512*424; i++)
                        {
                            depth[i] = frameData[i];
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();
                    }
                }
            }
        }

        /// <summary>
        /// Draws the depth image
        /// </summary>
        /// <param name="drawingContext">depth image to draw</param>
        private void DrawColorImage(DrawingContext drawingContext)
        {
            drawingContext.DrawImage(colorBitmap, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, new Pen(Brushes.YellowGreen, 8), jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="drawingContext"></param>
        private void DrawStandLine(DrawingContext drawingContext)
        {
            float Dfar = 4.0f;
            float Dnear = 3.0f;
            float Widehalf = 0.5f;
            CameraSpacePoint p1 = new CameraSpacePoint();
            p1.X = -Widehalf;
            p1.Z = Dfar;
            p1.Y = (-FloorPlane.W - (-Widehalf) * FloorPlane.X - (Dfar) * FloorPlane.Z) / FloorPlane.Y;

            CameraSpacePoint p2 = new CameraSpacePoint();
            p2.X = Widehalf;
            p2.Z = Dfar;
            p2.Y = (-FloorPlane.W - (Widehalf) * FloorPlane.X - (Dfar) * FloorPlane.Z) / FloorPlane.Y;

            CameraSpacePoint p3 = new CameraSpacePoint();
            p3.X = Widehalf;
            p3.Z = Dnear;
            p3.Y = (-FloorPlane.W - (Widehalf) * FloorPlane.X - (Dnear) * FloorPlane.Z) / FloorPlane.Y;

            CameraSpacePoint p4 = new CameraSpacePoint();
            p4.X = -Widehalf;
            p4.Z = Dnear;
            p4.Y = (-FloorPlane.W - (-Widehalf) * FloorPlane.X - (Dnear) * FloorPlane.Z) / FloorPlane.Y;

            ColorSpacePoint d1 = this.coordinateMapper.MapCameraPointToColorSpace(p1);
            ColorSpacePoint d2 = this.coordinateMapper.MapCameraPointToColorSpace(p2);
            ColorSpacePoint d3 = this.coordinateMapper.MapCameraPointToColorSpace(p3);
            ColorSpacePoint d4 = this.coordinateMapper.MapCameraPointToColorSpace(p4);

            SolidColorBrush br = Brushes.Red;
            if (HeightMeasure._state != "Start")
            {
                br = Brushes.Green;
            }

            drawingContext.DrawLine(new Pen(br, 8), new Point(d1.X, d2.Y), new Point(d2.X, d2.Y));
            drawingContext.DrawLine(new Pen(br, 8), new Point(d2.X, d2.Y), new Point(d3.X, d3.Y));
            drawingContext.DrawLine(new Pen(br, 8), new Point(d3.X, d3.Y), new Point(d4.X, d4.Y));
            drawingContext.DrawLine(new Pen(br, 8), new Point(d4.X, d4.Y), new Point(d1.X, d1.Y));
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="str"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <param name="dc"></param>
        private void DrawTextOnScreen(String str, int u, int v, DrawingContext dc)
        {
            if (str.Equals("Go"))
            {
                dc.DrawRectangle(Brushes.Green, null, new Rect(u - 50, v - 50, 520, 600));
            }
            else
            {
                dc.DrawRectangle(Brushes.Red, null, new Rect(u - 50, v - 50, 520, 600));
            }
            
            dc.DrawText(
               new FormattedText(str,
                  CultureInfo.GetCultureInfo("en-us"),
                  FlowDirection.LeftToRight,
                  new Typeface("Verdana"),
                  360, System.Windows.Media.Brushes.SeaShell),
                  new System.Windows.Point(u, v));
        }

        /// <summary>
        /// 
        /// </summary>
        private void DrawKeyFramesPicture()
        {
            if (HeightMeasure._state == "Done")
            {
                using (DrawingContext dc = this.drawingGroupInitLeave.Open())
                {
                    dc.DrawImage(HeightMeasure._initialLeaveBitmap, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }

                using (DrawingContext dc = this.drawingGroupPeakFlexion.Open())
                {
                    dc.DrawImage(HeightMeasure._peakFlexionBitmap, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
            else
            {
                using (DrawingContext dc = this.drawingGroupInitLeave.Open())
                {
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0, 0, this.displayWidth, this.displayHeight));
                }

                using (DrawingContext dc = this.drawingGroupPeakFlexion.Open())
                {
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0, 0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="stillness"></param>
        private void ControlRectByStillness(double stillness)
        {
            if (stillness > 0.05 || stillness < 0)
            {
                RectStillnessbar.Height = 432;
            }
            else
            {
                RectStillnessbar.Height = (int)(stillness / 0.05 * 432);
            }

            if (stillness < 0.003 && stillness > 0)
            {
                RectStillnessbar.Fill = Brushes.Green;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        private void UpdateJumpMeasureState()
        {
            int realTimeKnee = (int)HeightMeasure._realTimeKneeAngle;
            textBlock_KneeAngle.Text = "K: " + realTimeKnee.ToString();
            textblock_State.Text = HeightMeasure._state;

            if (HeightMeasure._state == "Done")
            {
                //const int constR = 1000;
                //var hinch = Math.Truncate(HeightMeasure.HeightByInch * constR) / constR;
                //var hm = Math.Truncate(HeightMeasure._Height * constR) / constR;
                //textblock_Height.Text = "Jumping Height (inch/m): " + hinch.ToString() + " / " + hm.ToString();
                //var sqt = Math.Truncate(HeightMeasure._eccentricPhase * constR) / constR;
                //textblock_SquattingTime.Text = "Eccentric Phase (ms): " + sqt.ToString();
                //var stt = Math.Truncate(HeightMeasure._concentricPhase * constR) / constR;
                //textblock_StretchTime.Text = "Concentric Phase (ms): " + stt.ToString();
                //var tr = Math.Truncate(HeightMeasure._timeRatio * constR) / constR;
                //textblock_TimeRadio.Text = "Time Ratio: " + tr.ToString();
                //var llv = Math.Truncate(HeightMeasure._PFLeftValgus * constR) / constR;
                //textblock_LeftValgus.Text = " -Left Valgus (deg): " + llv.ToString();
                //var lrv = Math.Truncate(HeightMeasure._PFRightValgus * constR) / constR;
                //textblock_RightValgus.Text = " -Right Valgus (deg): " + lrv.ToString();
                //var kasr = Math.Truncate(HeightMeasure._PFKASR * constR) / constR;
                //textblock_KASR.Text = " -Knee Ankle Separation Ratio: " + kasr.ToString();

                const int constR = 1000;
                var hheadm = Math.Truncate(HeightMeasure._HeightHead * constR) / constR;
                var hheadinch = Math.Truncate(HeightMeasure._HeightHead / 0.0254 * constR) / constR;
                var hchestm = Math.Truncate(HeightMeasure._HeightChest * constR) / constR;
                var hchestinch = Math.Truncate(HeightMeasure._HeightChest / 0.0254 * constR) / constR;
                var hspinebottomm = Math.Truncate(HeightMeasure._HeightSpineBottom * constR) / constR;
                var hspinebottominch = Math.Truncate(HeightMeasure._HeightSpineBottom / 0.0254 * constR) / constR;

                var ep = Math.Truncate(HeightMeasure._eccentricPhase * constR) / constR;
                var cp = Math.Truncate(HeightMeasure._concentricPhase * constR) / constR;
                var epcp = ep + cp;
                var tr = Math.Truncate(HeightMeasure._timeRatio * constR) / constR;
                var llv = Math.Truncate(HeightMeasure._PFLeftValgus * constR) / constR;
                var lrv = Math.Truncate(HeightMeasure._PFRightValgus * constR) / constR;
                var kasr = Math.Truncate(HeightMeasure._PFKASR * constR) / constR;

                textBox_Show.Text = "Jumping Height - Head (inch/m): " + hheadinch.ToString() + "/" + hheadm.ToString() + "\n" +
                                    "Jumping Height - Chest (inch/m): " + hchestinch.ToString() + "/" + hchestm.ToString() + "\n" +
                                    "Jumping Height - Spine Bottom (inch/m): " + hspinebottominch.ToString() + "/" + hspinebottomm.ToString() + "\n" +
                                    "Eccentric Phase (ms): " + ep.ToString() + "\n" +
                                    "Concentric Phase (ms): " + cp.ToString() + "\n" +
                                    "Total Time (ms): " + epcp.ToString() + "\n" +
                                    "Time Ratio (E/C): " + tr.ToString();
                textBox_Show2.Text = "PF Left Valgus (deg): " + llv.ToString() + "\n" +
                                     "PF Right Valgus (deg): " + lrv.ToString() + "\n" +
                                     "PF Knee-Ankle Separation Ratio: " + kasr.ToString();

                labelPF.Content = "Peak Flexion " + HeightMeasure.m_ST.ToString() + " " + HeightMeasure.m_PF.ToString();
                labelIL.Content = "Initial Leave " + HeightMeasure.m_IL.ToString();
            }
        }

        private void Button_Restart_Click(object sender, RoutedEventArgs e)
        {
            HeightMeasure = new HeightMeasurement();
            //textblock_Height.Text = "Jumping Height (inch/m): ";
            //textblock_SquattingTime.Text = "Eccentric Phase (ms): ";
            //textblock_StretchTime.Text = "Concentric Phase (ms): ";
            //textblock_TimeRadio.Text = "Time Ratio: ";
            //textblock_LeftValgus.Text = " -Left Valgus (deg): ";
            //textblock_RightValgus.Text = " -Right Valgus (deg): ";
            //textblock_KASR.Text = " -Knee Ankle Separation Ratio: ";
            TextBox_jumper_name.Text = "";
            textBlock_SaveAt.Text = "...";
            RectStillnessbar.Height = 432;
            RectStillnessbar.Fill = Brushes.Red;

            textBox_Show.Text = "";
            textBox_Show2.Text = "";
            labelPF.Content = "Peak Flexion ";
            labelIL.Content = "Initial Leave ";
        }

        private void Button_Save_Click(object sender, RoutedEventArgs e)
        {
            string _completeFilePath;
            string path = @"C:\Users\Public\Jumping Data";
            if (!Directory.Exists(path))
            {
                Directory.CreateDirectory(path);
            }
            _completeFilePath = @"C:\Users\Public\Jumping Data\" + TextBox_jumper_name.Text + ".csv";
            if (!File.Exists(_completeFilePath))
            {
                File.Create(_completeFilePath).Close();
            }
            string delimiter = "\t";
            string[][] output = new string[2][]{
                new string[]{"Time", "Name", "Jumping Height (Head)(inch)", "Jumping Height (Chest)(inch)", "Jumping Height (Spine Bottom)(inch)", 
                                "Eccentric Phase (ms)", "Concentric Phase (ms)", "Total Time (ms)",
                                "Time Ratio", "Left Valgus (at PF,deg)", "Right Valgus (at PF,deg)", "KASR (at PF)"},
                new string[]{System.DateTime.Now.ToString(), TextBox_jumper_name.Text, 
                             HeightMeasure._HeightHead.ToString(), HeightMeasure._HeightChest.ToString(),HeightMeasure._HeightSpineBottom.ToString(),
                             HeightMeasure._eccentricPhase.ToString(), HeightMeasure._concentricPhase.ToString(), (HeightMeasure._eccentricPhase + HeightMeasure._concentricPhase).ToString(),
                             HeightMeasure._timeRatio.ToString(), HeightMeasure._PFLeftValgus.ToString(), HeightMeasure._PFRightValgus.ToString(),
                             HeightMeasure._PFKASR.ToString()},

            };
            int length = output.GetLength(0);
            StringBuilder sb = new StringBuilder();
            for (int index = 0; index < length; index++)
            {
                sb.AppendLine(string.Join(delimiter, output[index]));
            }
            File.AppendAllText(_completeFilePath, sb.ToString());
            textBlock_SaveAt.Text = "Save at: " + path;

            HeightMeasure.SaveFrames();
        }
    }
}
