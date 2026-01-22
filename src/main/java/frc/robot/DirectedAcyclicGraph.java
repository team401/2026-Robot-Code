package frc.robot;

import java.io.PrintStream;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Written with the help of OpenAI ChatGPT for bug finding, algorithm explanations, and clarity
// suggestions

public class DirectedAcyclicGraph<T> {
  /** A map from each node to the list of its incoming nodes (direct predecessors) */
  private final Map<T, List<T>> incomingMap = new HashMap<>();

  /**
   * Add a new node into the graph
   *
   * @param node The node to add.
   */
  public void addNode(T node) {
    incomingMap.computeIfAbsent(node, k -> new ArrayList<>());
  }

  /**
   * Add a new edge to the graph, directed from tail to head.
   *
   * @param tail The "first" node, where the edge originates and the direct predecessor of head
   * @param head The "second" node, where the edge ends and the direct successor of tail
   */
  public void addEdge(T tail, T head) {
    incomingMap.computeIfAbsent(head, k -> new ArrayList<>()).add(tail);
    addNode(tail); // Ensure that the tail exists in the graph even if it isn't explicitly added
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
    // Make a new map to avoid mutating the old one
    Map<T, List<T>> incoming = new HashMap<>();
    for (var entry : incomingMap.entrySet()) {
      incoming.put(entry.getKey(), new ArrayList<>(entry.getValue()));
    }

    // First, find "start nodes" with no incoming edges
    Deque<T> startNodes = new ArrayDeque<>();

    for (T node : incoming.keySet()) {
      if (incoming.get(node).isEmpty()) {
        startNodes.add(node);
      }
    }

    // Next, while startNodes has items, take an item, remove all its outgoing edges, and then, if
    // any of the nodes its edges were removed from now have no incoming edges, add them to the
    // startNodes set.
    List<T> sortedList = new ArrayList<>();

    while (!startNodes.isEmpty()) {
      T node = startNodes.removeFirst();
      sortedList.add(node);
      for (T otherNode : incoming.keySet()) {
        if (incoming.get(otherNode).remove(node) && incoming.get(otherNode).isEmpty()) {
          startNodes.addLast(otherNode);
        }
      }
    }

    if (sortedList.size() != incomingMap.size()) {
      throw new IllegalStateException("Cyclic dependency detected in graph.");
    }

    return sortedList;
  }

  /**
   * Print the graph as a mermaid diagram
   *
   * @param printStream the PrintStream to print to, e.g. System.out
   */
  public void printMermaid(PrintStream printStream) {
    printStream.println("```mermaid");
    printStream.println("graph LR");
    for (var entry : incomingMap.entrySet()) {
      for (var incoming : entry.getValue()) {
        System.out.println(
            "  "
                + incoming.hashCode()
                + "(\""
                + incoming
                + "\") --> "
                + entry.getKey().hashCode()
                + "(\""
                + entry.getKey()
                + "\")");
      }
    }
    printStream.println("```");
  }
}
