#include "Measure/MeasureWeldParallelism.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using Points=std::vector<Eigen::Vector2d>;
double quantile(std::vector<double> values,double fraction)
{
    std::sort(values.begin(),values.end());
    const double index=fraction*(values.size()-1);
    const auto lower=static_cast<size_t>(index),upper=std::min(lower+1,values.size()-1);
    return values[lower]+(values[upper]-values[lower])*(index-lower);
}
Eigen::Vector2d direction(const Points& points,const std::vector<double>& weights,Eigen::Vector2d& center)
{
    double sum=0;center.setZero();
    for (size_t i=0;i<points.size();++i) { center+=weights[i]*points[i];sum+=weights[i]; }
    center/=sum;
    double xx=0,xy=0,yy=0;
    for (size_t i=0;i<points.size();++i) {
        const Eigen::Vector2d d=points[i]-center;
        xx+=weights[i]*d.x()*d.x();xy+=weights[i]*d.x()*d.y();yy+=weights[i]*d.y()*d.y();
    }
    const double angle=.5*std::atan2(2*xy,xx-yy);
    return {std::cos(angle),std::sin(angle)};
}
Eigen::Vector2d preliminaryDirection(const Points& points)
{
    std::vector<double> x,y,radii;
    for (const auto& p:points) { x.push_back(p.x());y.push_back(p.y()); }
    const Eigen::Vector2d middle(quantile(x,.5),quantile(y,.5));
    for (const auto& p:points) radii.push_back((p-middle).norm());
    const double radius=std::max(4*quantile(radii,.5),5.0);
    Points selected;
    for (const auto& p:points) if ((p-middle).norm()<=radius) selected.push_back(p);
    Eigen::Vector2d center;
    return direction(selected,std::vector<double>(selected.size(),1),center);
}
void fitLine(const Points& points,Eigen::Vector2d& center,Eigen::Vector2d& axis,double& rms)
{
    std::vector<double> weights(points.size(),1),residuals(points.size());
    for (int iteration=0;iteration<5;++iteration) {
        axis=direction(points,weights,center);
        const Eigen::Vector2d normal(-axis.y(),axis.x());
        for (size_t i=0;i<points.size();++i) residuals[i]=std::abs(normal.dot(points[i]-center));
        const double cutoff=std::max(.05,3*1.4826*quantile(residuals,.5));
        for (size_t i=0;i<points.size();++i) weights[i]=residuals[i]>cutoff ? cutoff/residuals[i]:1;
    }
    double error=0;
    for (const auto& p:points) error+=std::pow(axis.x()*(p.y()-center.y())-axis.y()*(p.x()-center.x()),2);
    rms=std::sqrt(error/points.size());
}
}

MeasureWeldParallelism::MeasureWeldParallelism(Cloud::ConstPtr first,Cloud::ConstPtr second,Cylinder cylinder)
    : clouds_{std::move(first),std::move(second)},cylinder_(std::move(cylinder)) {}

MeasureWeldParallelism::Result MeasureWeldParallelism::compute() const
{
    Result result;
    auto fail=[&](const std::string& message) { Result failed;failed.message=message;return failed; };
    auto cancelled=[&]() { Result stopped;stopped.cancelled=true;stopped.message="焊缝平行度测量已取消";return stopped; };
    auto progress=[&](int value,const std::string& stage) { return !progress_ || progress_(value,100,stage); };
    if (!progress(0,"准备测量")) return cancelled();
    if (!clouds_[0] || !clouds_[1] || clouds_[0]->size()<12 || clouds_[1]->size()<12)
        return fail("不可测: 两条焊缝的有效点不足");
    if (!result.frame.init(cylinder_.point,cylinder_.axis,cylinder_.radius)) return fail("不可测: 参考圆柱无效");
    std::vector<double> angles;
    for (const auto& cloud:clouds_) for (const auto& p:*cloud) {
        double phi;
        if (!result.frame.rawPhi(Eigen::Vector3d(p.x,p.y,p.z),phi)) return fail("不可测: 焊缝包含无效坐标");
        angles.push_back(phi);
    }
    result.frame.init(cylinder_.point,cylinder_.axis,cylinder_.radius,CylinderSurfaceFrame::choosePhi0(angles));
    double minimum=std::numeric_limits<double>::max(),maximum=-minimum;
    for (int side=0;side<2;++side) {
        std::vector<double> offsets;
        for (size_t i=0;i<clouds_[side]->size();++i) {
            if (i%256==0 && !progress(5+15*side+static_cast<int>(15.0*i/clouds_[side]->size()),"展开焊缝")) return cancelled();
            const auto& p=clouds_[side]->points[i];double a,s,e;
            result.frame.project(Eigen::Vector3d(p.x,p.y,p.z),a,s,e);
            result.projected[side].emplace_back(s,a);offsets.push_back(e);
            minimum=std::min(minimum,s);maximum=std::max(maximum,s);
        }
        result.radialOffset[side]=quantile(offsets,.5);
        if (std::abs(result.radialOffset[side])>std::max(20.0,.02*cylinder_.radius)) return fail("不可测: 焊缝与参考圆柱不对应");
    }
    if ((maximum-minimum)/cylinder_.radius>=std::acos(-1.0)) return fail("不可测: 焊缝周向跨度过大，请分段测量");
    if (!progress(40,"提取中心走向")) return cancelled();
    auto first=preliminaryDirection(result.projected[0]),second=preliminaryDirection(result.projected[1]);
    for (int side=0;side<2;++side) {
        const Eigen::Vector2d d=side==0 ? first:second,n(-d.y(),d.x());
        std::vector<double> alongCoordinates,acrossCoordinates;
        for (const auto& point:result.projected[side]) { alongCoordinates.push_back(point.dot(d));acrossCoordinates.push_back(point.dot(n)); }
        if (quantile(alongCoordinates,.95)-quantile(alongCoordinates,.05)<4*(quantile(acrossCoordinates,.95)-quantile(acrossCoordinates,.05)))
            return fail("不可测: 焊缝走向不明确，请选择单条焊缝");
    }
    if (first.dot(second)<0) second=-second;
    const Eigen::Vector2d along=(first+second).normalized();
    double lower=-std::numeric_limits<double>::max(),upper=-lower;
    for (const auto& points:result.projected) {
        std::vector<double> coordinates;
        for (const auto& p:points) coordinates.push_back(p.dot(along));
        lower=std::max(lower,quantile(coordinates,.02));upper=std::min(upper,quantile(coordinates,.98));
    }
    if (!(upper-lower>=12)) return fail("不可测: 两条焊缝没有足够的共同区段");
    const int bins=std::clamp(static_cast<int>((upper-lower)/2),6,200);
    const double step=(upper-lower)/bins;
    std::array<std::vector<std::vector<double>>,2> longitudinal;
    for (int side=0;side<2;++side) {
        longitudinal[side].resize(bins);
        for (const auto& p:result.projected[side]) {
            const double t=p.dot(along);const int bin=static_cast<int>(std::floor((t-lower)/step));
            if (bin<0 || bin>=bins) continue;
            longitudinal[side][bin].push_back(t);
        }
    }
    std::vector<int> best,current;
    for (int bin=0;bin<bins;++bin) {
        if (longitudinal[0][bin].size()<3 || longitudinal[1][bin].size()<3) continue;
        if (!current.empty() && bin-current.back()>3) { if (current.size()>best.size()) best=current;current.clear(); }
        current.push_back(bin);
    }
    if (current.size()>best.size()) best=current;
    if (best.size()<6) return fail("不可测: 两条焊缝的共同区段不连续或有效点不足");
    result.comparisonLength=(best.back()-best.front()+1)*step;
    for (int side=0;side<2;++side) {
        if (!progress(60+side*15,"拟合展开走向")) return cancelled();
        Eigen::Vector2d axis=side==0 ? first:second;
        const double begin=lower+best.front()*step,end=lower+(best.back()+1)*step;
        // 沿各自走向取中心，避免斜切宽点带引入角度偏差。
        for (int iteration=0;iteration<3;++iteration) {
            const Eigen::Vector2d normal(-axis.y(),axis.x());
            std::vector<double> offsets;
            for (const auto& p:result.projected[side]) offsets.push_back(p.dot(normal));
            const double offset=quantile(offsets,.5),projection=along.dot(axis);
            if (std::abs(projection)<1e-6) return fail("不可测: 无法确定焊缝走向");
            const double from=(begin-along.dot(normal)*offset)/projection,to=(end-along.dot(normal)*offset)/projection;
            const double start=std::min(from,to),width=std::abs(to-from)/best.size();
            std::vector<std::vector<double>> positions(best.size()),deviations(best.size());
            for (const auto& p:result.projected[side]) {
                const double t=p.dot(axis);const int bin=static_cast<int>(std::floor((t-start)/width));
                if (bin>=0 && bin<static_cast<int>(best.size())) { positions[bin].push_back(t);deviations[bin].push_back(p.dot(normal)); }
            }
            result.centers[side].clear();
            for (size_t bin=0;bin<best.size();++bin) if (positions[bin].size()>=3)
                result.centers[side].push_back(axis*quantile(positions[bin],.5)+normal*quantile(deviations[bin],.5));
            if (result.centers[side].size()<6) return fail("不可测: 两条焊缝的共同区段不连续或有效点不足");
            fitLine(result.centers[side],result.linePoint[side],result.lineDirection[side],result.centerlineRms[side]);
            axis=result.lineDirection[side];
        }
        if (result.centerlineRms[side]>std::max(.5,.03*result.comparisonLength)) return fail("不可测: 焊缝走向不明确，请缩小测量区段");
        const double denominator=along.dot(result.lineDirection[side]);
        if (std::abs(denominator)<1e-6) return fail("不可测: 无法确定焊缝走向");
        for (int i=0;i<=96;++i) {
            const double t=begin+(end-begin)*i/96;
            const Eigen::Vector2d p=result.linePoint[side]+result.lineDirection[side]*(t-along.dot(result.linePoint[side]))/denominator;
            result.curves[side].push_back(result.frame.unproject(p.y(),p.x(),result.radialOffset[side]));
        }
    }
    const auto& u=result.lineDirection[0];const auto& v=result.lineDirection[1];
    result.angleDegrees=std::atan2(std::abs(u.x()*v.y()-u.y()*v.x()),std::abs(u.dot(v)))*180/std::acos(-1.0);
    if (!progress(100,"完成测量")) return cancelled();
    result.success=true;
    return result;
}
