:- dynamic(kb/1).

makeKB(File):- open(File,read,Str),
               readK(Str,K),
               reformat(K,KB),
               asserta(kb(KB)),
               close(Str).

readK(Stream,[]):- at_end_of_stream(Stream),!.
readK(Stream,[X|L]):- read(Stream,X),
                      readK(Stream,L).

reformat([],[]).
reformat([end_of_file],[]) :- !.
reformat([:-(H,B)|L],[[H|BL]|R]) :- !,
                                    mkList(B,BL),
                                    reformat(L,R).
reformat([A|L],[[A]|R]) :- reformat(L,R).

mkList((X,T),[X|R]) :- !, mkList(T,R).
mkList(X,[X]).

initKB(File) :- retractall(kb(_)), makeKB(File).

astar(Node,Path,Cost) :- kb(KB), astar(Node,Path,Cost,KB).

arc([H|T],Node,Cost,KB) :- member([H|B],KB), append(B,T,Node),
  length(B,L), Cost is L+1.

arc([H|T],Node,KB) :- member([H|B],KB), append(B,T,Node).

heuristic(Node,H) :- length(Node,H).

goal([]).

%true if f(path1) <= f(path2)
less([[Node1|T1], Cost1], [[Node2|_], Cost2], [[Node1|T1], Cost1]) :-
  heuristic(Node1,Hvalue1), heuristic(Node2,Hvalue2),
  F1 is Cost1+Hvalue1, F2 is Cost2+Hvalue2,
  F1 =< F2.
%true if f(path1) > f(path2)
less([[Node1|_], Cost1], [[Node2|T2], Cost2], [[Node2|T2], Cost2]) :-
  heuristic(Node1,Hvalue1), heuristic(Node2,Hvalue2),
  F1 is Cost1+Hvalue1, F2 is Cost2+Hvalue2,
  F1 > F2.

%gets from a list of path-cost pairs the pair with the lowest f-value
get_lowest([], _, _).
get_lowest([H], _, H).
get_lowest([H1, H2|T], KB, Lowest) :-
  less(H1, H2, Less),
  get_lowest([Less|T], KB, Lowest).

%removes path-cost pair from list, returning new list in New
remove([], _, N, N).
remove([P|T], P, Temp, New) :- remove(T, P, Temp, New).
remove([H|T], P, Temp, New) :- remove(T, P, [H|Temp], New).

%takes in a path and the children of the head of the path and converts the
%children into path-cost pairs.
path_cost(_, [], _, L, L).
path_cost([[Node|R], NodeCost], [H|T], KB, List, L) :- arc(Node, H, C, KB),
  NewCost is NodeCost + C,
  path_cost([[Node|R], NodeCost], T, KB, [[[H, Node|R], NewCost]|List], L).

%adds path-cost pairs to Mother list
add_to_frontier(Mother, [], Mother).
add_to_frontier(Mother, [H|T], New) :- add_to_frontier([H|Mother], T, New).

%calls search predicate, which executes frontier search
astar(Node, Path, Cost, KB) :- search([[Node], 0], [], Path, Cost, KB).

%predicate to execute frontier search. Maintains a list of path-cost pairs
%in the second parameter. Elements of this list are removed when either all
%their children have been added or they have no children. Recursively calls
%itself, passing in the path-cost pair with the lowest f value each time and
%finding its children. Stops when a path has been found to the goal node.
search(_, [[[H|T], C]|_], [H|T], C, _) :- goal(H).

search([[Node|T], C], Path, FinalPath, Cost, KB) :-
  findall(X, arc(Node, X, _, KB), []),
  remove(Path, [[Node|T], C], [], NewList),
  get_lowest(NewList, KB, Lowest),
  search(Lowest, NewList, FinalPath, Cost, KB).

search([[Node|T], C], Path, FinalPath, Cost, KB) :-
  findall(X, arc(Node, X, _, KB), Children),
  path_cost([[Node|T], C], Children, KB, [], Pairs),
  add_to_frontier(Path, Pairs, NewPath),
  remove(NewPath, [[Node|T], C], [], NewPath2),
  get_lowest(NewPath2, KB, Lowest),
  search(Lowest, NewPath2, FinalPath, Cost, KB).
