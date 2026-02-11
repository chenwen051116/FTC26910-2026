package org.firstinspires.ftc.teamcode.auto;

import java.util.Date;
import java.util.function.BooleanSupplier;

public class Sequencer {
    private static class Node {
        private Runnable function;
        private BooleanSupplier condition;
        private int duration;
        private Node next;

        public Node(Runnable function) {
            this.function = function;
        }

        public Node(int timeInMs) {
            this.duration = timeInMs;
        }

        public Node(BooleanSupplier condition) {
            this.condition = condition;
        }
    }

    private Node head;
    protected Node tail;
    private Node currentNode;
    private long timeOfNextState;
    private boolean isRunning;
    private boolean hasRun;

    public Sequencer() {
        this.hasRun = false;
        this.isRunning = false;
    }

    public Sequencer run(Runnable function) {
        if (head == null) {
            head = tail = new Node(function);
        } else {
            tail = tail.next = new Node(function);
        }
        return this;
    }

    public Sequencer waitUntil(BooleanSupplier condition) {
        if (head == null) {
            head = tail = new Node(condition);
        } else {
            tail = tail.next = new Node(condition);
        }
        return this;
    }

    public Sequencer wait(int timeInMs) {
        if (head == null) {
            head = tail = new Node(timeInMs);
        } else {
            tail = tail.next = new Node(timeInMs);
        }
        return this;
    }

    public boolean begin() {
        if (hasRun) {
            return false;
        }
        hasRun = true;
        isRunning = true;
        currentNode = null;
        timeOfNextState = new Date().getTime();
        currentNode = head;
        return true;
    }

    public boolean update() {
        if (!isRunning || currentNode == null) {
            isRunning = false;
            return false;
        }

        long now = new Date().getTime();
        if (now < timeOfNextState) {
            return true;
        }

        if (currentNode.function != null) {
            currentNode.function.run();
            currentNode = currentNode.next;
        } else if (currentNode.condition != null) {
            if (currentNode.condition.getAsBoolean()) {
                currentNode = currentNode.next;
            }
        } else {
            timeOfNextState = now + currentNode.duration;
            currentNode = currentNode.next;
        }
        return true;
    }

    public void stop() {
        isRunning = false;
        hasRun = false;
    }
}
