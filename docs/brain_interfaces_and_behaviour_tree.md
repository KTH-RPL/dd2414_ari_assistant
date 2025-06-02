# Brain Interfaces and Behaviour tree

The nodes are run in a modular way, using behaviour trees and action servers. Most of the separate functionalities are implemented in a action server, such that they can be reused across different nodes.

The nodes are prioritized through a behaviour tree which is steered through the LLM. The LLM recognizes intents in the user’s speech, and publishes the intent to the behaviour tree. The behaviour tree then adds additional functionality to each intent, such that it can be activated through a request with an input, and attempted up to 10 times.

## Intent structure

Every intent has some infrastructure around it to make sure it is executed when requested. In addition, if the behaviour fails, a fail-counter in incremented, and after 10 failures the action is no longer attempted.

Here is an example of what the full intent tree looks like, in this case with the intent “explore”:

![explore](https://github.com/user-attachments/assets/66d642ab-36c2-4e89-94b2-3ce806c49fbf)


The first condition checks if the behaviour has been requested. This is checked via a blackboard value that gets updated through the intent callback function. If the behaviour has been requested, the failcounter is checked. A failcounter higher than 10 causes the action request to be cancelled and the counter to be reset. If the failconter is less than 10, the action is executed. If it fails, the counter is incremented. If it succeeds, the action request is removed and the failcounter reset.

## Behaviour tree structure

The entire behaviour tree can be found below:

![whole_tree](https://github.com/user-attachments/assets/601a332f-4bf7-4277-89f8-fb47e89425c7)

The blue nodes are intents, which contain the “intent structure” within the node. For clarity, the structure if left out of this tree. The tree is ticked every second from left to right. If an action is requested, the intent will be executed. Actions which are further left have higher priority, so they can interrupt an action further to the right. That way, a hierarchy of tasks is formed.

All the way on the left is the **stop** intent. If anyone says stop, any other intent is interrupted. In addition, the stop behaviour cancels any requests back to false, so ARI won’t return to a behaviour after the stop intent is completed.

All the way on the right is the **reject** intent. This intent causes ARI to say “I could not understand” if any prompt does not fit the intents. It is placed at lowest priority, which means that ARI will remain silent if it is in the process of executing an action.

## Setup

The behaviour tree is set up in the init function of the Brain node. Behaviours can either be created from action servers, or created directly as behaviours.

### Creating a behaviour as an action server

- **namespace\_dict**: Contains every intent and action server that is used within an intent. The namespace is used to save and publish the goal of the intent\*.
- **action\_dict:** Contains the behaviours corresponding to each intent. More complex behaviours such as sequences are defined before and passed as one behaviour in the Imaction\_dict.
- **StatusUpdate**: the action servers should be created as a StatusUpdate, which uses the BrainAction datastructure to communicate with the brain. See “move\_to\_poi.py” for an example on how to implement this. Here the phrase is also sent to ollama\_response such that ARI announces the action.

Note: The goal will be sent to the namespace defined for the intent. Every intent can only contain one input. For example, the “remember user” goal is sent to the face\_recognition\_node, since it uses the

\*The goals can not be changed dynamically once the behaviour tree is set up, which is why we are using additional publishers for the goal. This is dealt with in the StatusUpdate node.

### Creating a behaviour as a behaviour class

Another way of creating a behaviour is to create it as a behaviour class. As an example, see “explore” behaviour (exploration\_node.py). If the behaviour should be announced (“Initialising \_\_ action”), the namespace should be “ollama\_response” and the behaviour created as follows:

```
 explore_behaviour = py_trees.Sequence(
            "Explore", 
            [self.create_behaviour_from_action_server("explore"),
             ExploreBehaviour(name = "explore")])
```

If the behaviour does not need to be announced, it can simply be created as the behaviour class itself:

```
ExploreBehaviour(name = "explore")
```

The action and behaviour still need to be added to namespace\_dict and action\_dict.

### Hierarchy

The actions are prioritized according to the action\_dict list, from top to bottom.
