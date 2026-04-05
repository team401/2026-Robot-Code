package frc.robot.util;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class ServiceThread {
  private final BlockingQueue<Runnable> queue;
  private final Thread thread;
  private static final Runnable stopCommand = () -> {};
  public static ServiceThread defaultServiceThread =
      new ServiceThread("Default Service Thread", 10);

  public ServiceThread(String name, int len) {
    queue = new ArrayBlockingQueue<>(len);
    thread =
        new Thread(
            () -> {
              try {
                while (true) {
                  var command = queue.take();

                  if (command == stopCommand) {
                    break;
                  }

                  command.run();
                }
              } catch (InterruptedException e) {
                System.err.println(e);
              }
            },
            name);
    thread.start();
  }

  public void queueCommand(Runnable cmd) {
    queue.add(cmd);
  }

  public void shutdown() {
    queueCommand(stopCommand);
    try {
      thread.join();
    } catch (InterruptedException e) {
      System.err.println(e);
    }
  }
}
