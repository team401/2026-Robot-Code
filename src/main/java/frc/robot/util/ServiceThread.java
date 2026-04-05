package frc.robot.util;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class ServiceThread {
  private final BlockingQueue<Runnable> queue;
  private final Thread thread;
  private static final Runnable stopCommand = () -> {};
  
  /**
   * Default Service Thread with queue max of 100 commands.
   */
  public static ServiceThread defaultServiceThread =
      new ServiceThread("Default Service Thread", 100);

  /**
   * Creates a new Service thread.
   * @param name name of the thread
   * @param len maximum amount of commands that can be queued at once
   */
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

  /**
   * Adds a command to the queue of the service thread.
   * @param cmd the runnable that is queued
   * @return true if adding the command was successful (did not exceed queue capacity)
   */
  public boolean queueCommand(Runnable cmd) {
    return queue.offer(cmd);
  }

  /**
   * Stops the current service thread.
   */
  public void shutdown() {
    queueCommand(stopCommand);
    try {
      thread.join();
    } catch (InterruptedException e) {
      System.err.println(e);
    }
  }
}
