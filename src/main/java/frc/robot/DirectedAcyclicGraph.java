package frc.robot;

import java.io.PrintWriter;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Written with the help of OpenAI ChatGPT for bug finding, algorithm explanations, and clarity
// suggestions

public class DirectedAcyclicGraph<T> {
  /** A map from each node to the list of its outgoing nodes (direct successors) */
  private final Map<T, List<T>> successorMap = new HashMap<>();

  /**
   * Add a new node into the graph
   *
   * @param node The node to add.
   */
  public void addNode(T node) {
    successorMap.computeIfAbsent(node, k -> new ArrayList<>());
  }

  /**
   * Add a new edge to the graph, directed from tail to head.
   *
   * @param tail The "first" node, where the edge originates and the direct predecessor of head
   * @param head The "second" node, where the edge ends and the direct successor of tail
   */
  public void addEdge(T tail, T head) {
    successorMap.computeIfAbsent(tail, k -> new ArrayList<>()).add(head);
    addNode(head); // Ensure that the tail exists in the graph even if it isn't explicitly added
  }

  /**
   * Sort the graph topologically, such that for every edge (u, v) from node u to node v, u comes
   * before v in the ordering.
   *
   * <p>This method uses Kahn's Algorithm.
   *
   * @return An array of nodes, ordered topologically
   */
  public List<T> topologicalSort() {
    // Make a map to count the incoming edges of each node
    Map<T, Integer> incomingCountMap = new HashMap<>();

    for (var successorEntry : successorMap.entrySet()) {
      incomingCountMap.putIfAbsent(successorEntry.getKey(), 0);
      for (var outgoingNode : successorEntry.getValue()) {
        incomingCountMap.merge(outgoingNode, 1, Integer::sum);
      }
    }

    // First, find "start nodes" with no incoming edges
    Deque<T> startNodes = new ArrayDeque<>();

    for (var entry : incomingCountMap.entrySet()) {
      if (entry.getValue().intValue() == 0) {
        startNodes.add(entry.getKey());
      }
    }

    // Next, while startNodes has items, take an item, remove all its outgoing edges, and then, if
    // any of the nodes its edges were removed from now have no incoming edges, add them to the
    // startNodes set.
    List<T> sortedList = new ArrayList<>();

    while (!startNodes.isEmpty()) {
      T node = startNodes.removeFirst();
      sortedList.add(node);
      for (T otherNode : successorMap.get(node)) {
        int incomingCount = incomingCountMap.merge(otherNode, -1, Integer::sum);
        if (incomingCount == 0) {
          startNodes.addLast(otherNode);
        }
      }
    }

    if (sortedList.size() != successorMap.size()) {
      throw new IllegalStateException("Cyclic dependency detected in graph.");
    }

    return sortedList;
  }

  /**
   * Print the graph as a mermaid diagram
   *
   * @param printWriter the PrintWriter to print to
   */
  public void printMermaid(PrintWriter printWriter) {
    printWriter.println("```mermaid");
    printWriter.println("graph LR");
    for (var entry : successorMap.entrySet()) {
      if (entry.getValue().isEmpty()) {
        printWriter.println("  " + entry.getKey().hashCode() + "(\"" + entry.getKey() + ")\"");
      }
      for (var outgoing : entry.getValue()) {
        printWriter.println(
            "  "
                + entry.getKey().hashCode()
                + "(\""
                + entry.getKey()
                + "\") --> "
                + outgoing.hashCode()
                + "(\""
                + outgoing
                + "\")");
      }
    }
    printWriter.println("```");
  }

  public void printDOT(PrintWriter printWriter) {
    printWriter.println("digraph {");
    for (var entry : successorMap.entrySet()) {
      if (entry.getValue().isEmpty()) {
        printWriter.println("  \"" + entry.getKey() + "\"");
      }

      for (var outgoing : entry.getValue()) {
        printWriter.println("  " + "\"" + entry.getKey() + "\" -> " + "\"" + outgoing + "\"");
      }
    }
    printWriter.println("}");
  }
}
