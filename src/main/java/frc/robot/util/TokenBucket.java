package frc.robot.util;

public class TokenBucket {
  private final int maxCapacity;
  private final int incrementAmount;
  private int currentAmount;

  public TokenBucket(int maxCapacity, int incrementAmount) {
    this.maxCapacity = maxCapacity;
    this.incrementAmount = incrementAmount;
    this.currentAmount = maxCapacity;
  }

  public void increment() {
    currentAmount = Math.min(currentAmount + incrementAmount, maxCapacity);
  }

  public boolean consumeTokens(int numTokens) {
    if (currentAmount >= numTokens) {
      currentAmount -= numTokens;
      return true;
    }
    return false;
  }
}
