#pragma once
#include <ofxSoylent.h>
#include <SoyApp.h>
#include <TJob.h>
#include <TChannel.h>




class TPopOpencv : public TJobHandler, public TPopJobHandler, public TChannelManager
{
public:
	TPopOpencv();
	
	virtual bool	AddChannel(std::shared_ptr<TChannel> Channel) override;

	void			OnExit(TJobAndChannel& JobAndChannel);
	void			OnGetFeature(TJobAndChannel& JobAndChannel);
	void			OnFindFeature(TJobAndChannel& JobAndChannel);
	void			OnTrackFeatures(TJobAndChannel& JobAndChannel);
	void			OnFindInterestingFeatures(TJobAndChannel& JobAndChannel);
	void			OnNewFrame(TJobAndChannel& JobAndChannel);
	
public:
	Soy::Platform::TConsoleApp	mConsoleApp;
};



