import * as React from 'react';
import ROSLIB from "roslib";
import { RosConnection, rosImageSrcString, useRos } from 'rosreact';
import { useROS } from './ros-helpers';

function Camera() {
    const videoRef = React.useRef<HTMLVideoElement>(null);
    const videoAreaRef = React.useRef<HTMLDivElement>(null);

    return (
        <RosConnection url="ws://slinky.hcrlab.cs.washington.edu:9090" autoConnect>
            <div className='video-container' draggable={false}>
                <>
                    {/* <h4 className="title">Adjustable Camera</h4> */}
                    <div className="video-area" style={{ gridRow: 2, gridColumn: 1 }} ref={videoAreaRef}>
                        <div className={className("realsense-pan-tilt-grid", { constrainedHeight })}>
                            {panTiltButtons.map(dir => <PanTiltButton direction={dir} key={dir} />)}
                        </div>
                        <video
                            ref={videoRef}
                            autoPlay
                            muted={true}
                            className={className(videoClass, { constrainedHeight })}
                        />
                        {overlayContainer}
                    </div>
                </>
                <div className="under-video-area">
                    <UnderVideoButtons definition={definition}/>
                </div>
            </div>
        </RosConnection>
    );
}

export default Camera;