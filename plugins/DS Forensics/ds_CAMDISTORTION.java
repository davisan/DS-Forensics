import ij.*;
import ij.gui.*;
import ij.plugin.filter.*;
import ij.process.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.opencv.calib3d.*;
import java.util.*;
import org.codehaus.jettison.json.*;

/*
	Created by Davi Santos
*/
public class ds_CAMDISTORTION implements ExtendedPlugInFilter, DialogListener
{
	private static final String windowTitle = "DS Remove Camera Distortion";
    private final int FLAGS = DOES_8G | DOES_RGB | DOES_16 | DOES_32 | DOES_STACKS | PARALLELIZE_STACKS;
    private static boolean scaleParameters = true;
	private static double fx = 1;
	private static double fy = 1;
	private static double cx = 0;
	private static double cy = 0;
    private static double k1 = 0;
	private static double k2 = 0;
	private static double k3 = 0;
	private static double p1 = 0;
	private static double p2 = 0;
	private static double angle = 0;
	private static double scale = 1;
	private double k1Scale = 1;
	private double k2Scale = 1;
	private double k3Scale = 1;
	
	private int imageWidth = 0;
	private int imageHeight = 0;
	private int startWidth = 1;
	private int startHeight = 1;
	private TextField fxText;
	private TextField fyText;
	private TextField cxText;
	private TextField ctText;
	private TextField k1Text;
	private TextField k2Text;
	private TextField k3Text;
	private TextField p1Text;
	private TextField p2Text;
	
	private ImagePlus image = null;
	private Roi selectedPoints;
	private java.awt.Point[] selectedPointsArray = new java.awt.Point[0];
	
	String GetReport()
	{
		JSONObject log = new JSONObject();
		try
		{
			log.put("Plugin", windowTitle);
			JSONObject params = new JSONObject();
			params.put("Fx", fx);
			params.put("Fy", fy);
			params.put("Cx", cx);
			params.put("Cy", cy);
			params.put("K1", scaleParameters? k1*k1Scale:k1);
			params.put("K2", scaleParameters? k2*k2Scale:k2);
			params.put("K3", scaleParameters? k3*k3Scale:k3);
			params.put("P1", scaleParameters? p1*k1Scale:p1);
			params.put("P2", scaleParameters? p2*k1Scale:p2);
			params.put("Angle", angle);
			params.put("Scale", scale);
			params.put("Param Scale", scaleParameters);
			log.put("Parameters", params);
			return log.toString(4);
		}
		catch(Exception e) { return "ERROR!"; }
	}
	
	double LineAngle(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y)
	{	
		double a1 = Math.atan2(p1y-p2y, p1x-p2x);
		double a2 = Math.atan2(p2y-p3y, p2x-p3x);
		double result = Math.abs((a2-a1)*180/Math.PI);
		if(result > 180) result = 360 - result;
		return result;
	}
	Point2D.Double[] transformPoints(java.awt.Point[] points, double k1, double k2, double k3, double p1, double p2, double fx, double fy, double cx, double cy)
	{
		Mat camMat = new Mat(3,3, CvType.CV_32F);
		double[] tM = {fx,  0, cx, 
						0, fy, cy,
						0, 0, 1 };
		camMat.put(0,0, tM);
		Mat dist = new Mat(1,5, CvType.CV_32F);
		double[] tDist = { k1, k2, p1, p2, k3 };
		dist.put(0,0, tDist);
		
		java.util.List<org.opencv.core.Point> pList = new ArrayList<>();
		for(int i=0; i < points.length; ++i)
		{
			pList.add(new org.opencv.core.Point(points[i].x, points[i].y));
		}
		MatOfPoint2f pointsOCV = new MatOfPoint2f();
		pointsOCV.fromList(pList);
		MatOfPoint2f outPoints = new MatOfPoint2f();
		Calib3d.undistortPoints(pointsOCV, outPoints, camMat, dist, Mat.eye(3,3, CvType.CV_32F), camMat);
		
		org.opencv.core.Point[] undistortPoints = outPoints.toArray();
		Point2D.Double[] res = new Point2D.Double[points.length];
		for(int i=0; i < undistortPoints.length; ++i)
		{
			res[i] = new Point2D.Double(undistortPoints[i].x, undistortPoints[i].y);
		}
		return res;
	}
	String PrintPoints(Point2D.Double[] p)
	{
		String out = new String();
		for(int i=0; i < p.length; ++i)
			out += "(" + p[i].x + ", " + p[i].y + ")";
		return out;
	}
	String PrintPoints(java.awt.Point[] p)
	{
		String out = new String();
		for(int i=0; i < p.length; ++i)
			out += "(" + p[i].x + ", " + p[i].y + ")";
		return out;
	}
    
    @Override
    public int showDialog(ImagePlus imp, String command, PlugInFilterRunner pfr)
    {     
		image = imp;
		imageWidth = imp.getWidth();
		imageHeight = imp.getHeight();
		startWidth = imageWidth;
		startHeight = imageHeight;
		cx = (int) (startWidth*0.5);
		cy = (int) (startHeight*0.5);
		double maxRu=imageWidth*imageWidth/4 + imageHeight*imageHeight/4;
        maxRu=(double)Math.sqrt(maxRu);
        k1Scale = (maxRu*maxRu);
        k2Scale = (maxRu*maxRu*maxRu*maxRu);
		k3Scale = (maxRu*maxRu*maxRu*maxRu*maxRu*maxRu);
		k1 = k2 = k3 = p1 = p2 = 0;
	
        NonBlockingGenericDialog gd = new NonBlockingGenericDialog(windowTitle);
		gd.addNumericField("Focal Length X", fx, 16, 18, "");
		TextField fxText = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("Focal Length Y", fy, 16, 18, "");
		TextField fyText = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("Optical Center X", cx, 16, 18, "");
		TextField cxText = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("Optical Center Y", cy, 16, 18, "");
		TextField cyText = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("K1", k1, 16, 18, "");
		TextField k1Text = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("K2", k2, 16, 18, "");
		TextField k2Text = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("K3", k3, 16, 18, "");
		TextField k3Text = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("Vertical Perspective - P1", p1, 16, 18, "");
		TextField p1Text = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("Horizontal Perspective - P2", p2, 16, 18, "");
		TextField p2Text = (TextField) gd.getNumericFields().lastElement();
		gd.addNumericField("Rotation", angle, 4, 18, "");
		TextField angleText = (TextField) gd.getNumericFields().lastElement();
		gd.addSlider("Scale", 0.5, 1.5, scale, 0.01);
		TextField scaleText = (TextField) gd.getNumericFields().lastElement();
		
		Panel panel = new Panel();
		
		Button bEstimateK1 = new Button("Estimate K1"); 
		bEstimateK1.setBounds(0,0,180,30);  
		bEstimateK1.addActionListener(new ActionListener() {   
			public void actionPerformed(ActionEvent e){  
				java.awt.Point[] points = selectedPointsArray;
				if(points.length < 3) 
				{
					IJ.error("Must have a selection with a least 3 points.");
					return;
				}
				double bestK = 0;
				double bestError = points.length*180;

				double k = k1*(scaleParameters?k1Scale:1);
				double startVal = k - (k == 0? 1:Math.abs(0.4*k));
				double endVal   = k + (k == 0? 1:Math.abs(0.4*k));
				double stepVal = (endVal-startVal) / 100.0;
				int numIter = 0;
				Point2D.Double[] bestPoints = null;
				for(double iter=startVal; iter <= endVal; iter += stepVal)
				{
					if(numIter++ > 200) { IJ.log("loop"); break;}
					double error = 0;
					double testK = iter/(scaleParameters?k1Scale:1);
					Point2D.Double[] pointsprime = transformPoints(points, testK, k2, k3, p1, p2, fx, fy, cx, cy);
					for(int i=2; i < pointsprime.length; ++i)
					{
						error += Math.pow(LineAngle(pointsprime[i-2].x, pointsprime[i-2].y, pointsprime[i-1].x, pointsprime[i-1].y, pointsprime[i].x, pointsprime[i].y), 2);
					}
					if(error < bestError) 
					{ 
						bestK = testK; 
						bestError = error;
						bestPoints = pointsprime;
					}
				}
				bestK = (bestK*(scaleParameters?k1Scale:1));
				k1Text.setText(""+bestK);
		}});
		
		Button bReset = new Button("Reset"); 
		bReset.setBounds(0,0,180,30);  
		bReset.addActionListener(new ActionListener() {   
			public void actionPerformed(ActionEvent e){  
				cx = (int) (startWidth*0.5);
				cy = (int) (startHeight*0.5);
				fx = fy = 1;
				k1 = k2 = k3 = p1 = p2 = 0;
				angle = 0;
				scale = 1;
				fxText.setText("" + fx);
				fyText.setText("" + fy);
				cxText.setText("" + cx);
				cyText.setText("" + cy);
				k1Text.setText("" + k1);
				k2Text.setText("" + k2);
				k3Text.setText("" + k3);
				p1Text.setText("" + p1);
				p2Text.setText("" + p2);
				angleText.setText("" + angle);
				scaleText.setText("" + scale);
		}});  
		gd.addCheckbox("Scale Params", scaleParameters);
		gd.addToSameRow();
		panel.add(bEstimateK1);
		gd.addToSameRow();
		panel.add(bReset);
		gd.addPanel(panel);
		
		gd.addPreviewCheckbox(pfr);
        gd.addDialogListener(this);
		
        gd.showDialog();
        if (gd.wasCanceled())
        {
            return DONE;
        }
        else
        {            
			IJ.log(GetReport());
            return IJ.setupDialog(imp, FLAGS);
        }
    }
   
    @Override
    public boolean dialogItemChanged(GenericDialog gd, AWTEvent awte) 
    {
        fx = gd.getNextNumber();
        fy = gd.getNextNumber();
		cx = gd.getNextNumber();
        cy = gd.getNextNumber();
        k1 = gd.getNextNumber();
        k2 = gd.getNextNumber();
        k3 = gd.getNextNumber();
        p1 = gd.getNextNumber();
        p2 = gd.getNextNumber();
		angle = gd.getNextNumber();
		scale = gd.getNextNumber();
		scaleParameters = gd.getNextBoolean();
		
		if(scaleParameters) 
		{
			k1 /= k1Scale;
			k2 /= k2Scale;
			k3 /= k3Scale;
			p1 /= k1Scale;
			p2 /= k1Scale;
		}
		
		if(fx == 0) return false;
		if(fy == 0) return false;
		if(scale == 0) return false;

        IJ.showStatus(windowTitle);
        return true;
    }
    
    @Override
    public void setNPasses(int arg0)
    {
        
    }

    @Override
    public int setup(String arg0, ImagePlus imp)
    {
        if(!ds_LoadOCV.isLoad())
        {
            IJ.error("Library is not loaded.");
            return DONE;
        }

        if (imp == null)
        {
            IJ.noImage();
            return DONE;
        }
        else
        {
			selectedPoints = imp.getRoi();
			if(selectedPoints != null && selectedPoints.size() > 2)  
			{
				selectedPointsArray = selectedPoints.getContainedPoints();
			}
            return FLAGS;
        }
    }

    @Override
    public void run(ImageProcessor ip)
    {        
		try
		{
			if(ip.getBitDepth() == 8)
			{
				int imw = ip.getWidth();
				int imh = ip.getHeight();
				byte[] src_byte = (byte[])ip.getPixels();
				
				Mat src_mat = new Mat(imh, imw, CvType.CV_8UC1);            
				Mat dst_mat = new Mat(imh, imw, CvType.CV_8UC1);
				src_mat.put(0, 0, src_byte);
				Core.copyMakeBorder(src_mat, src_mat, imh/2, imh/2, imw/2, imw/2, Core.BORDER_CONSTANT);
				
				Mat camMat = new Mat(3,3, CvType.CV_32F);
				double[] tM = {fx,  0, cx+imw/2, 
								0, fy, cy+imh/2,
								0, 0, 1 };
				camMat.put(0,0, tM);
				
				Mat dist = new Mat(1,5, CvType.CV_32F);
				double[] tDist = { k1, k2, p1, p2, k3 };
				dist.put(0,0, tDist);
				Calib3d.undistort(src_mat, dst_mat, camMat, dist);
				Mat rotM = Imgproc.getRotationMatrix2D(new org.opencv.core.Point(cx+imw/2,cy+imh/2), angle, scale);
				Imgproc.warpAffine(dst_mat, dst_mat, rotM, new org.opencv.core.Size(2*imw, 2*imh), Imgproc.INTER_CUBIC);
				
				Mat cropped = new Mat(dst_mat, new org.opencv.core.Rect(imw/2, imh/2, imw,imh));
				cropped.get(0, 0, src_byte);
			}
			else if(ip.getBitDepth() == 16)
			{
				int imw = ip.getWidth();
				int imh = ip.getHeight();
				short[] src_short = (short[])ip.getPixels();
				
				Mat src_mat = new Mat(imh, imw, CvType.CV_16S);            
				Mat dst_mat = new Mat(imh, imw, CvType.CV_16S);
				src_mat.put(0, 0, src_short);
				Core.copyMakeBorder(src_mat, src_mat, imh/2, imh/2, imw/2, imw/2, Core.BORDER_CONSTANT);
				
				Mat camMat = new Mat(3,3, CvType.CV_32F);
				double[] tM = {fx,  0, cx+imw/2, 
								0, fy, cy+imh/2,
								0, 0, 1 };
				camMat.put(0,0, tM);
				
				Mat dist = new Mat(1,5, CvType.CV_32F);
				double[] tDist = { k1, k2, p1, p2, k3 };
				dist.put(0,0, tDist);
				Calib3d.undistort(src_mat, dst_mat, camMat, dist);
				Mat rotM = Imgproc.getRotationMatrix2D(new org.opencv.core.Point(cx+imw/2,cy+imh/2), angle, scale);
				Imgproc.warpAffine(dst_mat, dst_mat, rotM, new org.opencv.core.Size(2*imw, 2*imh), Imgproc.INTER_CUBIC);
				
				Mat cropped = new Mat(dst_mat, new org.opencv.core.Rect(imw/2, imh/2, imw,imh));
				cropped.get(0, 0, src_short);
			}
			else if(ip.getBitDepth() == 24)
			{
				int imw = ip.getWidth();
				int imh = ip.getHeight();
				int[] src_int = (int[])ip.getPixels();
				
				Mat src_mat = new Mat(imh, imw, CvType.CV_8UC3);
				Mat dst_mat = new Mat(imh, imw, CvType.CV_8UC3);
				ds_LoadOCV.intarray2mat(src_int, src_mat, imw, imh);
				Core.copyMakeBorder(src_mat, src_mat, imh/2, imh/2, imw/2, imw/2, Core.BORDER_CONSTANT);
				
				Mat camMat = new Mat(3,3, CvType.CV_32F);
				double[] tM = {fx,  0, cx+imw/2, 
								0, fy, cy+imh/2,
								0, 0, 1 };
				camMat.put(0,0, tM);
				
				Mat dist = new Mat(1,5, CvType.CV_32F);
				double[] tDist = { k1, k2, p1, p2, k3 };
				dist.put(0,0, tDist);
				Calib3d.undistort(src_mat, dst_mat, camMat, dist);
				Mat rotM = Imgproc.getRotationMatrix2D(new org.opencv.core.Point(cx+imw/2,cy+imh/2), angle, scale);
				Imgproc.warpAffine(dst_mat, dst_mat, rotM, new org.opencv.core.Size(2*imw, 2*imh), Imgproc.INTER_CUBIC);
				Mat cropped = new Mat(dst_mat, new org.opencv.core.Rect(imw/2, imh/2, imw,imh));
				ds_LoadOCV.mat2intarray(cropped, src_int, imw, imh);
			}
			else if(ip.getBitDepth() == 32)
			{
				int imw = ip.getWidth();
				int imh = ip.getHeight();
				float[] src_float = (float[])ip.getPixels();
				
				Mat src_mat = new Mat(imh, imw, CvType.CV_32F);            
				Mat dst_mat = new Mat(imh, imw, CvType.CV_32F);
				src_mat.put(0, 0, src_float);
				Core.copyMakeBorder(src_mat, src_mat, imh/2, imh/2, imw/2, imw/2, Core.BORDER_CONSTANT);
				
				Mat camMat = new Mat(3,3, CvType.CV_32F);
				double[] tM = {fx,  0, cx+imw/2, 
								0, fy, cy+imh/2,
								0, 0, 1 };
				camMat.put(0,0, tM);
				
				Mat dist = new Mat(1,5, CvType.CV_32F);
				double[] tDist = { k1, k2, p1, p2, k3 };
				dist.put(0,0, tDist);
				Calib3d.undistort(src_mat, dst_mat, camMat, dist);
				Mat rotM = Imgproc.getRotationMatrix2D(new org.opencv.core.Point(cx+imw/2,cy+imh/2), angle, scale);
				Imgproc.warpAffine(dst_mat, dst_mat, rotM, new org.opencv.core.Size(2*imw, 2*imh), Imgproc.INTER_CUBIC);
				Mat cropped = new Mat(dst_mat, new org.opencv.core.Rect(imw/2, imh/2, imw,imh));
				cropped.get(0, 0, src_float);
			}
			else
			{
				IJ.error("Wrong image format");
			}
			
			if(selectedPointsArray != null)
			{
				Point2D.Double[] pointsp = transformPoints(selectedPointsArray, k1, k2, k3, p1, p2, fx, fy, cx, cy);
				float[] xpoints = new float[pointsp.length];
				float[] ypoints = new float[pointsp.length];
				double ang = -angle*Math.PI/180;
				double rot11 = Math.cos(ang);
				double rot12 = -Math.sin(ang);
				double rot21 = Math.sin(ang);
				double rot22 = Math.cos(ang);
				for(int i=0; i < pointsp.length; ++i)
				{
					xpoints[i] = (float)(cx+(rot11*(pointsp[i].x-cx) + rot12*(pointsp[i].y-cy))*scale);
					ypoints[i] = (float)(cy+(rot21*(pointsp[i].x-cx) + rot22*(pointsp[i].y-cy))*scale);
				}
				Roi roi = new PointRoi(xpoints, ypoints);
				Overlay overlay = new Overlay();
				overlay.add(roi);
				if(image != null) {
					image.setRoi(roi);
				}
			}
		}
		catch(Exception e)
		{
			
		}
    }
}
