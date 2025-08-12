import React, { useEffect, useRef } from 'react';
import { useAppSelector } from '../store/hooks';

const MessageLog = ({ onClearLog }) => {
    const logRef = useRef(null);
    const logs = useAppSelector(state => state.logs?.entries || []); // Safe access with fallback

    useEffect(() => {
        if (logRef.current) {
            logRef.current.scrollTop = logRef.current.scrollHeight;
        }
    }, [logs]);

    return (
        <div className="panel" style={{ gridColumn: '1 / -1' }}>
            <h3>📋 Message Log</h3>
            <div className="log-container" ref={logRef}>
                {logs.length > 0 ? (
                    logs.map((log, index) => (
                        <div key={log.id || index} className="log-entry">
                            <span className="log-timestamp">[{log.timestamp}]</span>
                            <span>{log.message}</span>
                        </div>
                    ))
                ) : (
                    <div className="log-entry">
                        <span className="log-timestamp">[--:--:--]</span>
                        <span>No messages yet...</span>
                    </div>
                )}
            </div>
            <div style={{ marginTop: '10px' }}>
                <button className="btn btn-warning" onClick={onClearLog}>
                    Clear Log
                </button>
            </div>
        </div>
    );
};

export default MessageLog;