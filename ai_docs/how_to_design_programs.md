# A Step-by-step Template Driven Method of Good Programming Relying on Program Design

The goal of this document is to give a step-by-step description of how to systematically design programs.
We separate program design from implementation, and emphasize the design phase, of program design, function design, and feature design.

The gist of this document is in the Stepwise Definition section.


# Overview of what a Design Process

The design process within the designers mind is inherently hard to capture within rigorous formalism without killing it.
It should treated like a wild animal with an enormous home range within the designers thoughts, making unspoken and implicit design decisions and in general behaving in arbitrary ways.
Despite this, there are some things that the designer can do (besides hosting the process in his/her head):

 - Observe it
 - Describe it
 - Measure its outputs (exploring consequences, validation)
 - Moderately change its environment (goals, requirements, inspiration, knowledge)
 - Help it if it gets stuck

This methodology is intended to help designers do those things in a systematically in order to get the most out of a design processes.

## Search and Reinforcement Learning Analogy

All creative processes can be seen as searches in spaces of thinkable finished and unfinished results.
We'll view design processes as searches trying to arrive at the highest quality finished design with the least amount of effort.

The design process' guide during the search is a continuously updated estimate of what design quality it is heading towards.
Probability for a successful search is determined by both how accurate the estimates are and how the process makes use of them.
Within Reinforcement Learning, acting to improve estimate accuracy is called exploration and acting as if current estimation is good enough is called exploitation.
We need both exploration and exploitation in a design process.
Excluding one of them leads to random design decisions.

A case of pure exploration wouldbe to build something that you know will fail in order to learn from watching how it fails.

A case of pure exploitation would be to use a function in a way that was intended and thoroughly tested.

A Design process is building something that maybe fails, out of components that are known to work.

----------------

## The Designer and the Process

Design decisions are made continuously and are made based on ideas that arrive at seemingly random times.
We therefore separate The Design Process and The Designer.
The Process is a source of randomness continuously making an unknown number of small and big decisions.
The designers' work and responsibility is to:

 - Identify decisions and ideas
 - Describe decisions and ideas
 - Work out consequences and validate decisions and ideas
 - Try to guide and help the process from a distance
 - Help the process loose if it gets stuck

This way of externalizing seemingly unpredictable aspects of creative work has roots in ancient Greek and Roman tradition.

----------------

## Before Starting a Design Process

The designer should know how the quality of the product will be measured.
The estimate of what quality we're heading towards is the designer's compass in the vast space of thinkable designs and the design process gets confused without it.

Functional requirements and goals are good, a firm understanding of intended usage is better, running test ourselves is even better, and direct feedback from actual usage is best.

----------------

## Movements in Design Space

The design process traverses a vast decision tree while exploring the thinkable design space. That is, the design process steps through design space by making decisions.

The choice of concepts and design elements determines what kind of movements the process can take within design space.

For example the concepts "left half of machine" and "right half of machine" lets the process take a giant single step to complete machine. (Exploring "machines with two halves" might be a worthwhile exploration, but might not lead to good design quality in itself...)

The designer should help the design process is by making it aware of the current designs surroundings. For example, available design space gets limited by choices of

 - Which inputs it is designing to handle
 - Which primitive design elements it includes in the design
 - What language, concepts and higher order design elements it forms and uses

The designer should describe such decisions. The designer also has to care about questions like "do we need validation?" and "is it buildable?".

----------------

# The Designers' Job

The following is practical advice for helping the design processes and getting the most out of it in a systematic way.

----------------

## Keep A Log

All thoughts, identified decisions, reasoning around them, lessons learned and similar should be logged in chronological order to help debugging the process. Note what steps are exploration and what are exploitation.

----------------

## The Designer Routine
The following practical routine is intended to cover all the aspects of the designers' job. It should help the designer keep a systematic approach both in

 - Keeping documentation creation up to speed when the process moves quickly
 - Helping a derailed process get back on its feet

Our designer routine will use three sub-routines: Stepwise Definition, Input Definition, and Concept Creation.
These sub-routines recursively call each other, so the designers' systematic logging work will branch out according to how the process explores branches of the decision tree.
There is also a meta-routine called "Helping a Derailed or Halting Process".

A design process is preferably started out by entering Stepwise Definition at the highest possible level of abstraction.
If the process has already done progress in other directions, the designer should calm it down and describe what has already been done in terms of the sub-routine that is found to be most suitable.
What direction to guide the process next is described in the sub-routine.

Sub-routines may be applied to modules and sub-modules recursively.
That is, when a promising sub-design is found, we may allow the process to explore necessary sub-design and its sub-sub-designs.

When the search has to be rewinded, it is preferred to re-enter the decision tree and calling designer sub-routine in a best-first manner.
That is, consider the log of several previously run sub-routines before resuming the design process, and take note of what branch is being continued on/explored.

----------------

## Stepwise Definition: The Basic Steps of Program Design

0. Identify a problem that is reasonably isolated and small enough to think clearly about

Goal: Decide what's important.
We want the one problem statement that feels the most important at the current stage.
We want this written out in one or two sentences, in the language that feels the most natural.

How: Out of the list of all issues poking the designer's brain all at once, categorize them into "inessential details" and "core problem" or "essential problem".
Strip away all inessential details at first, write them down and put them away. Keep only the remaining core problem.
If the core problem is too big or vague to reason about, split it into sub-problem that can be handled one-by-one.
Then categorize the sub-problems into "inessential" and "essential" buckets.

For an AI agent, step 0 has often already been carried out by the user.
The most important problem, the one that we've chosen to focus on, is often already written out in the prompt.
If not, the AI agent should ask the user to do that.

1. From Problem Analysis to Data Definitions

Goal: Analyze the problem statement, typically stated as a word problem.
Describe how we've understood the design (sub)problem at hand.
We want descriptions the classes of data that go into the program or function and come out.

How: Give the thing we're going to design a name and describe the types of input and output (a "signature" or "contract").
Identify the information that must be represented and how it is represented in the chosen programming language. Formulate data definitions and illustrate them with examples.

2. Signature, Purpose Statement, Header

Goal: Extract and express the problem's essence, abstractly.
We want a concise purpose statement.

How: Reformulate eventual problem statements into a couple of sentences explaining the purpose of the thing we're designing.
State what kind of data the desired function consumes and produces. Formulate a concise answer to the question what the function computes. Define a stub that lives up to the signature.

3. Functional Examples

Goal: Illustrate the problem's essence with examples.
We want examples that illustrate the purpose statement and that serve as criteria for success.

How:	Illustrate the purpose statement by listing some example usages of the soon-to-be designed thing. Serve as criteria of success and are used as tests later.
Work through examples that illustrate the function’s purpose.

4. Function Template

Goal: Make outlines and plans based on the functional examples.
We want the givens to be organizing givens, into a template or inventory.

How: Some basic shape and logic of the design is already given at this stage. Write it down.
Translate the data definitions into an outline of the function.

5. Function Definition

Goal: Evaluate results with respect to expected outcomes.
Now we want some actual code.

How: Put together parts in ways that will solve the problem. If there's need for further specification of input go to Input Decision. Identify new sub-modules and list them for later stepwise definition.
Fill in the gaps in the function template. Exploit the purpose statement and the examples.

6. Testing

Goal: To be able to revise the overall product in light of failed checks and tests.
We want an actual working test suite.

How: Create a test suite from examples ensuring small inputs work as expected.
Articulate the examples as tests and ensure that the function passes all. Doing so discovers mistakes. Tests also supplement examples in that they help others read and understand the definition when the need arises—and it will arise for any serious program.

----------------

## Input Decision: Shrink Design Space by Narrowing the Range of Accepted Inputs

1. List available choices (that we can think of and express, and in reasonable detail)
2. If list seems unfinished: Go into Concept Creation
3. Explicitly reason around what's known, what's unknown and general pros and cons related to all the different choices
4. Write down the decision to explore
5. Write down how this limits the design space
6. Unless design step is purely exploratory: Validate that limitations doesn't violate requirements in any obvious way.

----------------

## Concept Creation: Make New Paths Through Design Space Accessible by Designing things that Seem Useful

1.	Dive into the space of potentially useful (for whatever reason) concepts.
2.	Do basic validation that the concept should be completely definable. Actually defining it through Stepwise Definition is recommended.
3.	If concept shows a lot of promise, line up all things that can be considered a sub-module and define them with the Stepwise Definition procedure as well.
4.	Review previous design process and re-enter it from where you estimate the best overall probability to arrive at a good design. The re-entrance is what makes the algorithm best-first.

----------------

## Helping a Derailed or Halting Process

Try jumping out of Stepwise Definition and into Concept Creation whenever

 - Language gets too formal to think clearly
 - Design progress is so slow that we loose track
 - Designing unused concepts that can potentially be better defined and built upon later is fine.

Try jumping out of Concept Creation and run under-defined concepts through Stepwise Definition whenever

 - Concepts become too random/har to think and reason about
 - The design starts to fail tests
 - Making tests more specific and reviewing previous validations carefully can also be of help.

----------------
