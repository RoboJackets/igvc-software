#pragma once

pcl::PointXYZ PointFromPixel(const cv::Point& pixel, const tf::Transform& cameraFrameToWorldFrame, image_geometry::PinholeCameraModel cam) {
    cv::Point3d cameraRay = cam.projectPixelTo3dRay(pixel);
    tf::Point worldCameraOrigin = cameraFrameToWorldFrame * tf::Vector3(0, 0, 0);
    tf::Point worldCameraStep = cameraFrameToWorldFrame * tf::Vector3(cameraRay.x, cameraRay.y, cameraRay.z) - worldCameraOrigin;
    double zScale = -worldCameraOrigin.z()/worldCameraStep.z();
    tf::Point ret = worldCameraOrigin + zScale * worldCameraStep;
    return pcl::PointXYZ(ret.x(), ret.y(), 0);
}

pcl::PointXYZ PointFromPixelNoCam(const cv::Point& p, int height, int width, double HFOV, double VFOV, double origin_z, double origin_y, double pitch) {
    int xP = p.x;
    int yP = p.y + (height / 2 - 100);

    double pitch_offset = ((float) (yP - height / 2) / height) * VFOV;
    double y = origin_z /tan(pitch + pitch_offset) + origin_y;

    double theta = ((float) (xP - width / 2) / width) * HFOV;
    double x = y * tan(theta);
    return pcl::PointXYZ(x, y, 0);
}

double toRadians(double degrees) {
    return degrees / 180.0 * M_PI;
}