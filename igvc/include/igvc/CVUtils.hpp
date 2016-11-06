#pragma once

pcl::PointXYZ PointFromPixel(const cv::Point& pixel, const tf::Transform& cameraFrameToWorldFrame, image_geometry::PinholeCameraModel cam) {
    cv::Point3d cameraRay = cam.projectPixelTo3dRay(pixel);
    tf::Point worldCameraOrigin = cameraFrameToWorldFrame * tf::Vector3(0, 0, 0);
    tf::Point worldCameraStep = cameraFrameToWorldFrame * tf::Vector3(cameraRay.x, cameraRay.y, cameraRay.z) - worldCameraOrigin;
    double zScale = -worldCameraOrigin.z()/worldCameraStep.z();
    tf::Point ret = worldCameraOrigin + zScale * worldCameraStep;
    return pcl::PointXYZ(ret.x(), ret.y(), 0);
}