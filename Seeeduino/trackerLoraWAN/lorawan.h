extern void lorawansetup(void);
extern void lorawanloop(void);
extern void lorawanusercommand(const char *line);

// Link data rate management
extern int currentDR;
extern int gwmargin;   // Margin on gateway end
extern int pendingLCR;   // Number of LCR messages sent since last response received
extern float lastSNR;  // SNR of last received message
extern int lastRSSI;  // RSSI of last received message
extern unsigned long lastReceived;  // Time (msec) of last received message
extern unsigned long lastLCR;  // Time (msec) of last LCR message received
extern char devAddr[];  // Device address
extern int ulcntr, dlcntr;
