package frc.lib.util;

import java.util.List;

public class DataProcessor implements Runnable {
  public interface ReaderLoggerData {
    void readLogDataFromIO();
  }

  public interface IODataRefresher {
    void refreshData();
  }

  public static final int LOOP_TIME = 20;

  public static void initDataProcessor(
      ReaderLoggerData readerLogger, IODataRefresher... dataRefreshers) {
    new Thread(new DataProcessor(readerLogger, dataRefreshers)).start();
  }

  private long timestamp = 0;
  private ReaderLoggerData dataReaderLogger;
  private List<IODataRefresher> iODataRefreshers;

  public DataProcessor(ReaderLoggerData readerLogger, IODataRefresher... dataRefreshers) {
    this.dataReaderLogger = readerLogger;
    iODataRefreshers = List.of(dataRefreshers);
  }

  public void run() {
    while (true) {
      timestamp = System.currentTimeMillis();
      for (IODataRefresher i : iODataRefreshers) {
        i.refreshData();
      }

      dataReaderLogger.readLogDataFromIO();
      try {
        var delta = System.currentTimeMillis() - timestamp;
        if (delta < LOOP_TIME) {
          Thread.sleep((long) (LOOP_TIME - delta));
        }
      } catch (InterruptedException e) {
      }
    }
  }
}
