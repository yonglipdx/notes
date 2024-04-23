'''cpp
escape specail chars: + - . !  [ ] ( ) | < > " ' & | ` #
<pre>...</pre> 
<code>...</code>
'''

```cpp
vim block  insert   (exc twice)
1. select  block area with  ctr-v
2. shift i or  I
3 insert context
4. exc TIWCE 

save,append and copy linee in vim(with buffer a) 
1. \"a3+y    --> yank 3 lines in a
2. \"A8+y    --> append(A) 5 lines into a
3. \"ap      --> paste above 11 lines 

save, copy, (not append) lines to clipbord
clipboard register for linux is "\*" for window is "+"
1. copy: hightlight and   \"\*y"              (linux),  \"\+y\"   (window)
2  paste                  mouse-middle-click(linux),  ctrl\-v (window) 

rcording macro:   q  qq  @
1. q + register(a-z)
2. operations
3. qq to save
use macro
 @ register(a-z)

Change: cw/ciw   cf/ct    cnf/cnt    ci + '({""  r/#r/R/s/#s/a/A o/O x/dd 
yanking: y w|iw|$|^|%{([ paste: P|p
move:    H/M/L/ZZ w/e/b 
search:  / ? + n/N
copy:    #start,#end2#destination

cw - changes the word from the cursor to the end of the word 
ciw - changes the entire worth that the cursor is on  (i--> entire)
cf<char> - changes text until you find the character <char>, includes the find char
ct<char> - changes text until you find the character <char>, but don't include the char
c<n>f<char> - same as previous cf but find the nth occurrence of the char
c<n>t<char> - same as previous ct but find the nth occurrence of the char
ci( - change text inside current brackets ( )
ci{ - change text inside current curly braces { }
ci' - change text inside current ' quotes
ci" - change text inside current " quotes
cit - change text inside current html/xml tag
r<char> - replaces the character under the cursor
<n>r<char> - replaces the next <n> characters from the cursor with <char>
R - overwrite mode / Replace characters
s - substitute, remove the character under cursor and enter insert mode
<n>s - remove the next <n> characters and enter insert mode
o - open ( leave in insert mode) a new line under current line with correct indenting
O - same as o, but open above
x - delete character
dd - delete line
u - undo
A - append to end of current line ( leaves in insert mode )
a - append after current cursor (leaves in insert mode )
Navigation
h - cursor left
j - cursor down
k - cursor up
l - cursor down
H - put cursor at top of screen
M - put cursor in middle of screen
L - put cursor at bottom of screen
w - beginning of next word
e - end of next word
b - beginning of previous word, if in the middle of a word, it goes back to the beginning of that word
gd - goto definition ( use on top of methods / classes etc )
zz - center current line in center of screen
* - search for word under cursor
m<char> - mark current location and store it in <char> if the letter is a Capital letter, then it works across files
 `<char> - goto mark set in <char> if the letter is a capital letter it will jump to the file with the mark
Searching
Use it to navigate to places faster
/ search forward, by itself repeats the last search but forwards
? search back, by itself repeats the last search but backwards
n find next, will go to the next forward or the next back ( depends on whether you used / or ?)
N find previous, will go to the previous forward or the previous back
From the .vsvimrc bindings
[ - Previous Method (triggers R#)
] - Next Method (triggers R#) 
```

```cpp
/*
You use dynamic_cast to cast a pointer/RERENCE from a base class to a derived class if you have polymorphic classes
(virutal desctructior is needed) and want to perform a safe downcast.

For casting from a subclass to its superclass, you can use a regular cast (static_cast) because it's always a valid conversion.

static_cast is a more general casting. pointer, refernce, object, built-in. It's the programmer's responsibility
 to make that the casting is valid. So you can blindly replace all dynamic_cast with static_cast in the codebase
 without introduing build issure during compile time.
But it is preferred to use dynmaic_cast + success/failure(null) check + optional try/catch block to dispatch logic.


In this example, Base is a polymorphic class with a virtual function, and Derived is derived from Base.
We create a Derived object and assign its address to a Base*. Later, we use dynamic_cast to safely downcast the Base* to a Derived*.
If the object is indeed of the derived type, the dynamic_cast will succeed, and we can safely call functions specific
 to the derived class using the resulting pointer (derivedPtr).
If the cast fails (for example, if the object is not of the derived type), dynamic_cast returns a null pointer.

Be aware that if static_cast is operating on object by value, a NEW object would be created as usual with the possibility of slicing.

*/

#include <iostream>

class Base {
public:
    virtual void baseFunction() {
        std::cout << "Base class function" << std::endl;
    }

    virtual ~Base() {} // A virtual destructor is necessary for polymorphic classes
};

class Derived : public Base {
public:
    void derivedFunction() {
        std::cout << "Derived class function" << std::endl;
    }
};

int main() {
    Base* basePtr = new Derived();  // Create a Derived object and assign its address to a Base pointer

    // Use dynamic_cast to safely downcast to a Derived pointer
    Derived* derivedPtr = dynamic_cast<Derived*>(basePtr);

    // pointer downcasting
    if (derivedPtr) {
        // The dynamic_cast succeeded, use derivedPtr to call Derived class functions
        derivedPtr->derivedFunction();
    } else {
        // The dynamic_cast failed, likely because the object isn't of the expected type
        std::cout << "Dynamic cast failed" << std::endl;
    }

    delete basePtr;
    return 0;
}

// for reference downcasting
Base& baseRef = someDerivedObject;
try {
    Derived& derivedRef = dynamic_cast<Derived&>(baseRef);
    // ...
} catch (std::bad_cast& e) {
    // Handle the case where the cast fails
}
 

```


## chain of responsibility (COR)
```cpp
#include <vector>
#include <iostream>
using namespace std; 

struct Goblin;
struct GoblinKing; 
struct Game;

struct StatQuery
{
  enum Statistic { attack, defense } statistic;
  StatQuery(Statistic at_or_defense, int v):statistic(at_or_defense), result(v){};
  int result {0};
};

struct Creature
{ 
  Game& game;
  int base_attack, base_defense;

public:
  Creature(Game &game, int base_attack, int base_defense) : game(game), base_attack(base_attack),
                                                            base_defense(base_defense) {}
  virtual int get_attack() = 0;
  virtual int get_defense() = 0;
};


struct Game
{
  vector<Creature*> creatures;
  void attach(Creature* cr){ 
      creatures.push_back(cr);
  }  
  
  void process(StatQuery& sq, Creature* cr)
  { 
      switch(sq.statistic)
      {
          case sq.Statistic::attack:
            sq.result = cr->base_attack;
            for (auto c: creatures){
                if (c == cr) continue; 
                if (c->base_attack == 3){
                    sq.result +=1;
                } 
            }
            break;
          case sq.Statistic::defense:
            sq.result = cr->base_defense;
            for (auto c: creatures){
                if (c == cr) continue;
                sq.result+=1;
            }
            break;
      }; 
  }
};



class Goblin : public Creature
{
public:
  Goblin(Game &game, int base_attack, int base_defense) : Creature(game, base_attack, base_defense) {}

  Goblin(Game &game) : Creature(game, 1, 1) {game.attach(this);} 

  int get_attack() override {
    StatQuery task(StatQuery::attack, 0);
    game.process(task, this);
    return task.result;
  }

  int get_defense() override {
    StatQuery task(StatQuery::defense, 0);
    game.process(task, this);
    return task.result;
  }
};

class GoblinKing : public Goblin
{
public:
  GoblinKing(Game &game) : Goblin(game, 3, 3) {  }

  // todo
};


int main(){
  Game g;
  Goblin* g1 = new Goblin(g);
  Goblin* g2 = new Goblin(g);
  Goblin* g3 = new Goblin(g);
  GoblinKing* gk1 = new GoblinKing(g);
  cout << g1->get_attack() << endl;
  cout << g1->get_defense() << endl;
  cout << g3->get_attack() << endl;
  cout << g3->get_defense() << endl;
  cout << gk1->get_attack() << endl;
  cout << gk1->get_defense() << endl;
  return 0;
}


```


## Observer cat attack

```

#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
 
struct IRat
{
    virtual void update(size_t count) = 0;
};


struct Game
{

    vector<IRat*> users{};
    void attach(IRat* rat)
    {
        users.push_back(rat);
        notify();
    }

    void deattach(IRat* rat)
    {
        users.erase(std::remove(users.begin(), users.end(), rat));
        notify();
    }
    
    void notify()
    {
        for (auto r : users)
        {
          r->update(users.size());
        }
    }
    
};

struct Rat : public IRat{

    Game& game;
    int attack{1};

    // work
    Rat(Game& g):game(g)
    {
      game.attach(this);
    }

//   doesn't work:     
//    Rat(Game& g)
//    {
//      game = g;
//      game.attach(this);
//    }
//   
    
    void update(size_t count) override {
        attack = (int)count;
    } 

    ~Rat() 
    { 
       game.deattach(this);
    }

};



int main(){
   Game g;
   Rat* r1   = new Rat(g);
   cout << r1->attack << endl;
  {
   Rat* r2   = new Rat(g);
   cout << r2->attack << endl;
   cout << r1->attack << endl;
   delete r2;
  }
  cout << r1->attack << endl;

}


```

## Mediator pattern (was above observer pattern is Mediator as well? )
```

#include <vector>

using namespace std;

struct IParticipant {
    int value  {0};
};

struct Mediator
{
    vector<IParticipant*> participants;
};


struct Participant : IParticipant
{
    // int value{0};
    Mediator& mediator;

    Participant(Mediator &mediator) : mediator(mediator)
    {
      mediator.participants.push_back(this);
    }

    void say(int value)
    {
        cout << this << " say " << value << endl;
        for (auto& p : mediator.participants){
            if ( dynamic_cast<IParticipant*>(this) == p) continue;
            p->value += value;
        }
    }
};

```

## VirtualBox with widows share folder

> su -  "login with root"
> sudo usermod -aG sudo <username> 
> sudo usermod -aG vboxsf <username>
> virtualBox shared folder set:  auto mount / persistant / onto /mnt/c .. 
> sudo apt-get update
> sudo apt-get install virtualbox-guest-dkms (not found)
> sudo apt-get install virtualbox-guest-utils


## Install chrome
> wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
> sudo dpkg -i google-chrome-stable_current_amd64.deb
sudo apt install -f
google-chrome 

## vim mouse switch split terminals
> set mouse=a

## install boost/gtest ask chatGPT

## cmake for observer pattern with gest and boost
```cpp
cmake_minimum_required(VERSION 3.6)
project(DesignPatternDemos)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

find_package(Boost COMPONENTS REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(GTest REQUIRED)
include_directories(${GTest_INCLUDE_DIR})

set(SOURCE_FILES main.cpp headers.hpp Observer.hpp SaferObservable.hpp)
add_executable(DesignPatternDemos ${SOURCE_FILES})

target_link_libraries(DesignPatternDemos GTest::GTest GTest::Main Boost::boost)
target_include_directories(DesignPatternDemos PRIVATE ${GTEST_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

cmake -DCMAKE_BUILD_TYPE=Debug CMakeLists.txt ----> debug build
``` 

## Files
```cpp
// SafterObservable.hpp

#pragma once

#include <string>
#include <vector>
#include <mutex>

template <typename> struct Observer;

template <typename T>
struct SaferObservable
{
  std::vector<Observer<T>*> observers;
  typedef std::recursive_mutex mutex_t;
  mutex_t mtx;
public:
  void notify(T& source, const std::string& field_name)
  {
    std::scoped_lock<mutex_t> lock{mtx};
    for (auto observer : observers)
      if (observer)
        observer->field_changed(source, field_name);
  }

  void subscribe(Observer<T>& observer)
  {
    std::scoped_lock<mutex_t> lock{mtx};
    observers.push_back(&observer);
  }

  void unsubscribe(Observer<T>& observer)
  {
    auto it = std::find(begin(observers), end(observers), &observer);
    if (it != end(observers))
      *it = nullptr;
//    std::scoped_lock<mutex_t> lock{mtx};
//    observers.erase(
//      remove(observers.begin(), observers.end(), &observer),
//      observers.end()
//    );
  }
};

// Observer.hpp

#pragma once
#include <string>

template <typename T>
struct Observer
{
  virtual void field_changed(
    T& source,
    const std::string& field_name
  ) = 0;
};

// Main.cpp

#include "Headers.hpp"
#include "Observer.hpp"
#include "SaferObservable.hpp"

class Person : public SaferObservable<Person>
{
  int age{0};
public:
  Person(){}
  Person(int age) : age(age) {}

  int get_age() const
  {
    return age;
  }

  void set_age(int age)
  {
    if (this->age == age) return;

    auto old_can_vote = get_can_vote();
    this->age = age;
    notify(*this, "age");

    if (old_can_vote != get_can_vote())
      notify(*this, "can_vote");
  }

  bool get_can_vote() const {
    return age >= 16;
  }
};

// observer & observable

struct ConsolePersonObserver
  : public Observer<Person>
{
private:
  void field_changed(Person &source, const std::string &field_name) override
  {
    cout << "Person's " << field_name << " has changed to ";
    if (field_name == "age") cout << source.get_age();
    if (field_name == "can_vote")
      cout << boolalpha << source.get_can_vote();
    cout << ".\n";
  }
};

struct TrafficAdministration : Observer<Person>
{
  void field_changed(Person &source, const std::string &field_name) override
  {
    if (field_name == "age")
    {
      if (source.get_age() < 17)
        cout << "Whoa there, you're not old enough to drive!\n";
      else
      {
        cout << "Oh, ok, we no longer care!\n";
        source.unsubscribe(*this);
      }
    }
  }
};

int main(int ac, char* av[])
{
  Person p, p2;
  TrafficAdministration ta;
  p.subscribe(ta);
  p2.subscribe(ta);

  p.set_age(15);
  p.set_age(16);
  p2.set_age(11);
  p2.set_age(17);
  try
  {
    p.set_age(17);
  }
  catch (const std::exception& e)
  {
    cout << "Oops, " << e.what() << "\n";
  }

  return 0;
}

```

## Q: Can't have both F(const int x) and F(int x) ##

[meaningful const in function declaration]("https://abseil.io/tips/109")
```cpp

// Two declaration --> essentially the same as const is ignored in declaration. But compile OK 
void F(int);                     // 1: declaration of F(int)
void F(const int);               // 2: re-declaration of F(int)

// Two defination is NOT allowed below
void F(int) { /* ... */ }        // 3: definition of F(int)
void F(const int) { /* ... */ }  // 4: error: re-definition of F(int)


//GOOD: Non-top level const
void F(const int* x);                  // 1
void F(const int& x);                  // 2
void F(std::unique_ptr<const int> x);  // 3
void F(int* x);                        // 4


//Top-level const  ---> should avoid
void F(int x) {};            // 1: declares F(int)
//void F(const int x) {};            // 1: declares F(int)

void F(int* x) {};         // 2: declares F(int*)
//void F(int* const x) {};         // 2: declares F(int*)

void F(const int* x);   // 3: declares F(const int*)
// void F(const int* const x);   // 3: declares F(const int*)



void F(const int* x) {};                  // 1
void F(const int& x) {};                  // 2
void F(std::unique_ptr<const int> x) {};  // 3
void F(int* x){};  // 4


void F(const int x);          // 1: declares F(int)
void F(int* const x);         // 2: declares F(int*)
void F(const int* const x);   // 3: declares F(const int*)
```



[const visual studio]("https://learn.microsoft.com/en-us/cpp/cpp/const-cpp?view=msvc-170)


###androit phone debug mode ###
> setting -> about phone   -> build number -> muliple tap 
> system  -> development option -> usb debugging ....

[androit phone reset](https://en-us.support.motorola.com/app/answers/detail/a_id/163512/p/30,2660,4598,#:~:text=With%20the%20phone%20powered%20off,until%20the%20device%20turns%20on.&text=Press%20the%20Power%20button%20to,Power%20button%20to%20select%20it)


## recheck the statemeant: 6.9
> In order for variables to be usable in compile-time contexts, 
> such as array sizes, the compiler has to see the variable’s definition (==not just a forward declaration==).

> Q: what is the best naming prefix for a global non-const variable
> A: //

> Q: What’s the difference between a variable’s scope, duration, and linkage? What kind of scope, duration, and linkage do global variables have?
>   Scope determines where a variable is accessible. Duration determines when a variable is created and destroyed. Linkage determines whether the variable can be exported to another file or not.
>   Global variables have global scope (aka. file scope), which means they can be accessed from the point of declaration to the end of the file in which they are declared.
>   Global variables have static duration, which means they are created when the program is started, and destroyed when it ends.
>   Global variables can have either internal or external linkage, via the static and extern keywords respectively.

> function are default to external linkage. can use *static* to set it as internal linkage
> to use global function from another file, make forward declaration in **another** file
> use extern for const glabal                                 --> add **extern** in ori file and forward declaration with *extern* in another file
> non-const and constexpr global val are external by default  --> extern optional            and forward declaration with *extern* in another file

```cpp
 //creater.cpp
  // global variable definitions
int g_x { 2 }; // non-constant globals have external linkage by default
extern const int g_y { 3 }; // this extern gives g_y external linkage
```

```cpp
user.cpp
#include <iostream>
extern int g_x; // this extern is a forward declaration of a variable named g_x that is defined somewhere else
extern const int g_y; // this extern is a forward declaration of a const variable named g_y that is defined somewhere else

int main()
{
    std::cout << g_x << '\n'; // prints 2

    return 0;
```

> gloabl varibale and function identifier can have either internal linkage or external linkage
> const/constexpr globals have internal linkage by default 
> to make non-constant globals internal, we use the **static** keyword
> *storage class specifier* sets both name's linkage and its storage duration (but not its scope). ==static, extern, mutable==  

> avoid varibel shadowing
> -Wshadow compiler flag (gcc/clang) to avod unintended varaible shadowing

> global varaible has file scope and static duration
> global variable is initialized by defualt (unlike local variable)
> using non-constant global variable should be generally be avoid altogether


> if one symbol can't be found in current space, it will check each containing namespace in sequence utill global
> can only be define in file scope (including nested) but not a function. e.g. main()
> it's ok to declare namespace in multiple locations: different files or different place in the same file. 
> since c++17 namespace can be added namespace foo::goo {...}

### If using std::getline() to read strings, use std::cin >> std::ws input manipulator to ignore leading whitespace.


> need 0b (binary) in bits{0b0000'0101} below defination
> std::bitset<8> bits{ 0b0000'0101 }; // we need 8 bits, start with bit pattern 0000 0101
```cpp

test() allows us to query whether a bit is a 0 or 1
set() allows us to turn a bit on (this will do nothing if the bit is already on)
reset() allows us to turn a bit off (this will do nothing if the bit is already off)
flip() allows us to flip a bit value from a 0 to a 1 or vice versa

std::bitset<8> bits{ 0b0000'0101 }; // we need 8 bits, start with bit pattern 0000 0101
bits.set(3); // set bit position 3 to 1 (now we have 0000 1101)
bits.flip(4); // flip bit 4 (now we have 0001 1101)
bits.reset(4); // set bit 4 back to 0 (now we have 0000 1101)



```

*string_view*
: constant string since C++17

```cpp
 int choice{};
std::cin >> choice;
std::cout << "Now enter your name: ";
std::string name{};
std::getline(std::cin >> std::ws, name); // note: added std::ws here
```


##  Best practice ##
##Prefer literal suffix L (upper case) over l (lower case).##
```cpp
Data type	Suffix	Meaning
integral	u or U	unsigned int
integral	l or L	long
integral	ul, uL, Ul, UL, lu, lU, Lu, or LU	unsigned long
integral	ll or LL	long long
integral	ull, uLL, Ull, ULL, llu, llU, LLu, or LLU	unsigned long long
integral	z or Z	The signed version of std::size_t (C++23)
integral	uz or UZ	std::size_t (C++23)
floating point	f or F	float
floating point	l or L	long double
string	s	std::string
string	sv	std::string_view
```

##Q: when print/use float, does it round to most significant digits?##

```cpp

constexpr
: Any variable that should not be modifiable after initialization and whose initializer is known at compile-time should be declared as constexpr.
NO for function parameter

const
: Any variable that should not be modifiable after initialization and whose initializer is not known at compile-time should be declared as const.
Yes for function parameter

int x{5}; // 5 means integer
double y{5.0}; // 5.0 is a floating point literal (no suffix means double type by default)
float z{5.0f}; // 5.0 is a floating point literal, f suffix means float type

How to convert numbers to scientific notation
Use the following procedure:
Your exponent starts at zero.
Slide the decimal so there is only one non-zero digit to the left of the decimal.
Each place you slide the decimal to the left increases the exponent by 1.
Each place you slide the decimal to the right decreases the exponent by 1.
Trim off any leading zeros (on the left end of the significandet
Trim off any trailing zeros (on the right end of the significand) only if the original number had no decimal point. 
We’re assuming they’re not significant unless otherwise specified.

start with: 600.410
Slide decimal left 2 spaces: 6.00410e2
No leading zeros to trim: 6.00410e2
Don't trim trailing zeros: 6.00410e2 (6 significant digits)

However, in C++, 87 and 87.000 are treated exactly the same!


c++17 [[maybe_unused]] int i {5};

std::endl vs ‘\n’

Using std::endl can be a bit inefficient, as it actually does two jobs: it moves the cursor to the next line of the console, 
and it flushes the buffer. When writing text to the console, we typically don’t need to flush the buffer at the end of each line. 
It’s more efficient to let the system flush itself periodically (which it has been designed to do efficiently)

variable/function name: start with lower case. 

Forward declarations can also be used to define our functions in an order-agnostic manner
Less often, there are times when we have two functions that call each other. Forward declarations give us a way to resolve 
such circular dependencies.

#ifndef SQUARE_H
#define SQUARE_H

int getSquareSides(); // forward declaration for getSquareSides
int getSquarePerimeter(int sideLength); // forward declaration for getSquarePerimeter

#endif

#pragma once
For advanced readers

There is one known case where #pragma once will typically fail. If a header file is copied so that it exists in multiple places 
on the file system, if somehow both copies of the header get included, header guards will successfully de-dupe 
the identical headers, but #pragma once won’t (because the compiler won’t realize they are actually identical content).






C++ question:
The Questions handled in the course is as follows:

Where are the function declarations found
Where are the function definitions found
After compilation, are the function definitions found in a.out file
In static library, are the library files part of the executable file (a.out)
In dynamic library, are the library files part of the executable file (a.out)
What is the full form of a.out
What is the format of a.out file
Is reference a constant pointer
What happens when a reference variable is re-assigned a value from another variable 
What happens when a reference variable is used with & and is re-assigned a value from another variable
What is the difference between a reference and constant pointer
What is the advantage of a reference over a pointer
How does a reference to a reference work
In the given program, convert all the const pointer statements to references
Explain the below statements:
    int *const p;
    const int *p;
    const int * const p;
    int const *p;
What is the difference between the operators (&) and (&*) when used in the context of pointers
In C++, is it mandatory to name a structure or a union
Is the life time of a locally declared reference limited to the function in which it is declared
Declare the following:
    an array of 5 integers
    an array of 5 pointers
    a pointer to an array of 5 integers
    a reference to an array of 5 integers
Is it possible to create an array of references
Can we have multiple definitions of a function in a single module
What is the difference between declaration and definition of a variable
Define a structure which contains a member function to add 2 of it’s data members and print the result
Does a pointer to a variable act like a constant reference OR Does a reference to a variable act like constant pointer
A reference can point to different variables but a variable can’t be tied to multiple references
Does scope resolution operator help us in retrieving only the value of global variables whose identifier names are similar to the local variable names
What is the difference between the below two statements:
   a constant reference 
   reference is a constant pointer
Can we call a pointer ( which points to a read only string ) as a constant pointer
Do we get a lvalue error in an assignment statement where a function call is placed on the left hand side of the assignment operator
Are there only 2 ways to pass variables to a function ?
Does passing arguments by reference really advantageous
Is static variable intermodular or intramodular
Are Inline functions similar to macro definitions
Function overloading is asking the same function to perform different tasks in different contexts of the program
Name mangling is the concept where a name management database is used to maintain different global variables across different modules
Can we overload the function to accept an char value and an char reference
Suppose I have a structure having data members. I create 2 structure objects. Can I add 2 structure objects which automatically will take care of adding the data members of the 2 objects of the structure
Can we increment the data member of the structure through the structure object using the ++ operator
Operator overloading is programming the operator to perform a different function
In function overloading, Is name mangling concept applied on both function definitions and declarations 
Is name mangling concept taken care for all the functions irrespective of whether the function is overloaded or not
Function overloading is differentiating the function( with same names) based on their return type
Can A function be overloaded only once
We will get a compilation error, if we overload --operator to perform incrementation
Macros perform typechecking of the arguments passed to macro functions
What is a class and a object
Is it true that A constructor is not a mandatory function in a class
Is it true that A destructor function is automatically called when the object of the class is constructed
Is it true that when Malloc allocates memory dynamically when creating an object of the class, it also calls the constructor
Is it true that when New allocates memory dynamically when creating an object of the class, it doesn’t call the constructor
Is it true that when Delete operator calls the destructor function of the class when the object is being destroyed
Is it true that when Free function can be used to free the memory which is dynamically allocated by using the new operator
Is it true that when Delete operator can be used to free the memory which is dynamically allocated by using the new operator
Can A public data member of the class can be accessed from anywhere with in the class or from outside the class
Can A private data member of the class can be accessed by the class object from outside the class
Can A member function of the class can be made private
Can A public member function of a particular class can be called by the object of the class from outside the class
Can A private member function cannot be called directly by the object of the class from outside the class
Can I alter the private data member of the class from outside the class
Is it true that A static member function can be accessed only by the class name from outside the class
Can a static member function change the data member value belonging to a particular object
Can a constructor of a object initialize the static data member of the class
Can a normal member function change modify the static data member of the class
What is the difference between a static member function accessing the static data member and a normal member function accessing the static data member
Can we modify a publicly declared static data member from outside the class
Can one class contain an another class in it
Create an array of 3 objects and initialize them
Is it logical to initialize the data members in the class when the class is created
Can we initialize the public const data member of the class from outside the class
Can one class have an other class’s object declared in it’s private section
Can we create an object of an anonymous/nameless class 
Does an overloaded assignment operator get’s called by default when one object is assigned to the other object
Does an overloaded ‘+’ operator get’s called by default when one object is added with another object
Can we assign an integer value to an object 
Explain the concept of this pointer
Is it true that the member functions can only access the data members related to class
Can we create a static object
Can we have a const member function
Is there any difference between assignment operators and copy constructors 
Which function get’s called when an object is passed to a function or returned from a function by value
Is it true that If a derived class is privately derived from a base class then the protected and public data members of the base class will become private for the desired class
Is it true that If a derived class is protectedly derived from a base class, then protected and public data members of the base class will become protected for the derived class
Is it true that If a derived class inherits privately all public members of the base class, the public member of the base class will become private for the derived class 
Continued from the above statement, can we make some of the members of the base class to be accessible from outside the derived class
Is the size of the base class is equal to the sum of the sizes of the data members of the base class and the derived classes 
Is it true that multiple inheritance is similar to multilevel inheritance
Can we derive a child class from a base class even before the base class is defined
A derived class is derived publicly from the base class. The derived class has got a member function which overrides the member function 
defined in the base class . Is it true that the member function of the base class will be called by an object of the derived class
Is it true that the base class can access the public members of the derived class
Is it true that When a class is publicly derived from a base class, then the private members of the base class is not accessible from outside the derived class, but the public and protected members are accessible from outside and inside the member functions of the derived class
Is it true that When a class is privately derived from a base class, the protected and public members of the base class will become private for the derived class 
Is it true that When a class is protectedly derived from a base class, then the protected and public members of the base class become protected members of the derived class 
Is it true that The object of the derived class can access the private and protected members of the base class from outside the derived class
Is it true that a virtual function without the expression = 0 added to the declaration can’t be overridden
Is it true that An abstract class is something where in the derived class can’t override the member functions of the base class
Is it true that we can’t have a pointer to an abstract class
Is it true that The normal functions are resolved at run time
Is it true that the virtual table contain the address of virtual functions
Is it true that The address of the derived class object can’t be stored in a base class pointer. This leads to a compilation error
Is it true that Virtual functions don’t support polymorphism
Can a derived class pointer can call the virtual function defined in the base class 
Is it true that The virtual function table is shared across different objects of the class
Is it true that The this pointer contains the address of the object where as the object contains vptr which in turn holds the address of the virtual table
Can we have a pure virtual constructor and destructor functions
Is it true that An abstract base class is something which doesn’t contain a pure virtual function
Is it true that An abstract class doesn’t need a constructor
Is it true that The ‘has a’ is a concept where in when a class is derived from another class, it is said to a type of it’s base class 
Is it true that The ‘kind of’ relationship is something where in one class will have an object of another class created in it
Can we use the explicit keyword along with the constructors to prevent implicit conversions
Is it true that The data member of the const object can be changed using the mutable keyword
Is it true that a friend function of a class can never access private data of a class
Is it true that a friend function should mandatorily a member function of the class
Is it true that Forward referencing is a concept where in a friend function can refer a undefined and a undeclared class
Is it true that A friend class of a class can access the data members of the class
Is it true that The dynamic cast operator gives the information about the object at run time
Can we convert a const integer variable to non const variable
Is it true that We don’t have to rewrite the code when we are using composition or inheritance
Is it true that A function that is a friend of a class can access only the public data members of a class
Is it true that If class B is declared as a friend to class A, then class A can access all private & public members of class B
Is it true that templates can’t be used on user defined data type
Is it true that Function template and template functions are similar
Is it true that Macros are more advantageous then templates
Is it true that We get a compilation error if class templates are instantiated using objects
Is it true that The member functions of a template class should mandatorily be defined in the class
Is it true that If there is a function template called fun(), then a specific appropriate version of it would be generated when the fun() is called with a new data type argument
Is it true that template saves memory
Is it true that A function template should mandatorily have only one argument type
Is it true that We get run time error if we set default values to template arguments
Is it true that We get compile time error if function templates are overloaded 




create/use dynamic library
cc -o libfile.so --shared file.o

export LD_LIBRARY_PATH=`pwd` -lfile
gcc -I./ -L`pwd1 main.c -lfile

```








```cpp

// CUDA error handling
cudaError error;
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

// CHECK cude memory transfer runtime
clock_t mem_dtoh_start, mem_dtoh_end;
mem_dtoh_start = clock();
gpuErrchk(cudaMemcpy(gpu_result, d_c, NO_BYTES, cudaMemcpyDeviceToHost));
mem_dtoh_end = clock();
printf("Mem transfer device to host : %4.6f \n",
		(double)((double)(mem_dtoh_end - mem_dtoh_start) / CLOCKS_PER_SEC));

int blockId = blockIdx.z * gridDim.y * gridDim.x + blockIdx.y * gridDim.x + blockIdx.x;
int blockSize = blockDim.x * blockDim.y * blockDim.z;
int idx = blockSize * blockId + threadIdx.z * blockDim.y * blockDim.x + threadIdx.y * blockDim.x + threadIdx.x;
```

A kernel is a parallel function that  can be launched directly from the host (the CPU) on device (the GPU)
A device function is a function that can only be called from a kernel function or another device function.
A device function look and act like normal serial C/C++ functions, only they are running on the GPU and are 
called in parallel from kernels. 

## 
```python
device_data = gpuarray.to_gpu(host_data)
from_device = device_data.get()

seq = np.array([1,100,-3,-10000, 4, 10000, 66, 14, 21],dtype=np.int32)
seq_gpu = gpuarray.to_gpu(seq)
max_gpu = InclusiveScanKernel(np.int32, "a > b ? a : b")
print max_gpu(seq_gpu).get()[-1]
print np.max(seq)
```


```cpp
polymorphism
  1. compile time: method overloading, operator overloading
  2. run time: function overloading, virtual funciton.

constructor has the same name as class. no return type. 
public: all data member and member function are accessible 

/*
class B : public A {...}
inheritance is private by default
when you privately inherit from a class or struct, you explicitly say,
among other things, that direct conversion from a derived class to a
base type isn't possible
*/

class A {
public:
	virtual	void say() {
		std::cout << "from A\n";
	}
private:
 int A_pr = 100;

friend void access_A();
};

class B : public A { 
public:
	void say() {
		std::cout << "from B\n";
	}
private:
 int B_pr;
};

void access_A(){
  A a;
  // friend void access_A() is needed in class A
  std::cout << a.A_pr << std::endl;
}


int main() {
	A* a = new A();  // A a2;  ALL OK
	a->say();        // a2.say(); 
	B* b = new B();  // B b2;
	b-> say();       // b2.say();
	A* c = new B();  //A& c2 = b2;
	c->say();       // c2.say();
  access_A();   // friend needed
	return 0;
}

```

[flowers and persons testing2]( https://leetcode.com/problems/number-of-flowers-in-full-bloom/description/ )
```cpp
  vector<int> fullBloomFlowers(vector<vector<int>>& flowers, vector<int>& persons) {
        vector<int> ret(persons.size());
        map<int,vector<int>> day_person;
        for (int i=0; i<persons.size(); i++) day_person[persons[i]].push_back(i);
        priority_queue<int, vector<int>,greater<int>> PQ;
        sort(flowers.begin(),flowers.end());
        int f_idx = 0;
        for (auto& d : day_person)
        {
            auto& cur_d = d.first;
            auto& ps = d.second;
            while (f_idx < flowers.size() && flowers[f_idx][0] <= cur_d)
            {
                while (PQ.size() && PQ.top() < cur_d) PQ.pop();
                PQ.push(flowers[f_idx++][1]);
            }
            while (PQ.size() && PQ.top() < cur_d) PQ.pop();
            for (auto p: ps) ret[p] = PQ.size();
        }
        return ret;
    } 

```

```python

 def fullBloomFlowers(self, flowers: List[List[int]], persons: List[int]) -> List[int]:
        flowers.sort()
        fIdx, pq, ret = 0 , [], [0]*len(persons)
        for pIdx, dIdx in sorted(enumerate(persons), key=lambda a: a[1]):
            while fIdx < len(flowers) and flowers[fIdx][0] <= dIdx:
                heapq.heappush(pq, flowers[fIdx][1])
                fIdx+=1
            while pq and pq[0] < dIdx: heapq.heappop(pq)
            ret[pIdx] = len(pq)
        return ret
 
```

```cpp
  vector<int> fullBloomFlowers(vector<vector<int>>& flowers, vector<int>& persons) {
        vector<int> ret;
        vector<int> start, end;
        for (auto& f : flowers){
            start.push_back(f[0]);
            end.push_back(f[1]);
        }
        sort(start.begin(),start.end());
        sort(end.begin(),end.end());

        for (auto p: persons){
            auto idx2 = upper_bound(start.begin(), start.end(), p)-start.begin();
            auto idx1 = lower_bound(end.begin(), end.end(), p)-end.begin();
            ret.push_back(idx2-idx1);
        }
        
        return ret;


```
```python
 def fullBloomFlowers(self, flowers: List[List[int]], persons: List[int]) -> List[int]:
        start,end = [sorted([a for a,b in flowers]), sorted([b for a,b in flowers])]
        return [bisect_right(start, p) - bisect_left(end,p) for p in persons]

```


```cpp
Ways to create threads in c++11
  1. Function pointers
  2. Lambda functions
  3. Functors
  4. Member functions
  5. Static member functions 

Race condtion creates undefined behaviro.
  1. Not all operations are atomic.
  2. Opeartion get interleaved.
  3. Half reading.   xxxxx.
Due to optimizations, the actual executed code might be different..

std::mutex g_mtx;
g_mtx.lock()
...
g_mtx.unlock();

Resource Acquisition is initialization. (RAII)
std::lock_guard<std::mutex> lg(g_mtx);
std::unique_lock<std::mutex> ul<g-mtx>  (can unlock() and lock() again)
std::shared_lock<std::mutex> ul<g-mtx>  (eg. multiple reading).
std::scoped_lock sl(g_mtx1, g_mtx2)

condition_variable.

std::mutex g_mtx;
std::condtion_varaible g_cv;
bool g_ready = false;

void producer(){
  while (true){
    unique_lock<std::mutex> ul(g_mtx);
    g_ready = true;
    ul.unlock();
    g_cv.notify_one();
    un.lock();
    g_cv.wait(ul, [](){return g_ready == false;})
  }
}

void producer(){
  while(true){
    unique_lock<std::mutex> ul(g_mtx);
    g_ready = true;
    // if wait() failed.  ul.unlock() will be called.
    // if wait() successed. un.lock() will be called.
    g_cv.wait(ul, [](){return g_ready;}
    // consume data.
    g_ready = false;
    g_cv.notify_one();
    ul.lock();
    g_cv.wait(ul, [](){return !g_ready;})
  }

lock provides: atomic +   sequence synchronization. 

```


Array/Vector has to be contiguous.
For vector, when adding new element and it doesn't preserve the feature of contigous, the WHOLE vector will be copies and put onto 
another BIGGER place which has large "contigous" space.  

**generate one commit patch from git**

git format-patch -1 <sha>   (git)
patch -p1 < xxx.patch (perforce)

**tempalte class: constructors and function overload**

```cpp
#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;

template<class T>
class Complex
{
public:
	T real, imag;
	Complex(T r, T i):real(r), imag(i){};
	Complex(const Complex<T>& other) { real = other.real; imag = other.imag; };

	const Complex& operator=(const Complex<T>& other){
	  real = other.real; imag = other.imag;
	  return *this;
	}

	bool operator<(const Complex<T>& other) const {
		return other.real * other.real + other.imag * other.imag > real*real + imag*imag;
	};

	template<class U>  // Have to redefine the template
	friend ostream& operator << (ostream&, const Complex<U>&); // this is not member. cant be const funct.
};

template<class T>
ostream& operator <<(ostream& os, const Complex<T>& c){
	os << c.real << "/" << c.imag;
	return os;
}

int main(){
	vector<Complex<int>> V;
	for (int i=-5; i<=5; i++) V.push_back(Complex<int>(i,i));
	//V[0] = Complex<int>(1000, 1000);  // assign construction
	V[0] = V[1] = Complex<int>(1000, 1000);  // assign construction
	sort(V.begin(), V.end());
	for (auto v : V) cout << v << endl;
	Complex<int>tmp(V[5]);
	cout << tmp << endl;
	return 0;
}


```


**c_function_pointer, cpp_functor, cpp_bind_placeholders, cpp_functional, cpp_lambda**

```cpp

#include <iostream>
#include <functional>

using namespace std;


int add(int a, int b){
   return b-a;
}

/* c function pointer */
int c_fun_pointer(int a, int b, int (*pfun)(int,int)){
   return pfun(a,b);
}

/* cpp functor */
struct cpp_functor
{
	int operator()(int a, int b) const {
		return a+b;
	}
};

/* cpp functional */
int cpp_functional(int a, int b, function<int(int,int)> func){
   return func(a, b);
}

class OP{
public:
  int minus (int a, int b){
    return a-b;
  }
};

int main() {

   int a = 111, b = 222;
   cout << c_fun_pointer(a,b,add) << endl;    // c function pointer;
   cpp_functor my_functor;
   cout << my_functor(a,b) << endl;  // cpp functor with instance name;
   cout << cpp_functor()(a,b) << endl; // cpp functor without instance name;

   auto tmp_fun = [](int a, int b){return a+b;}; // lambda
   cout << tmp_fun(a,b) << endl;

   auto bind_func1 = bind(add, placeholders::_1, placeholders::_2);
   cout << bind_func1(100,200) << endl;
   auto bind_func2 = bind(add, placeholders::_2, placeholders::_1);
   cout << bind_func2(100,200) << endl;
   auto bind_func3 = bind(add, placeholders::_1, 1000);
   cout << bind_func3(200) << endl;


   OP op;  // must need an instance ?  ===> YL
   auto bind_func_member = bind(&OP::minus, op, 1000, placeholders::_1);
   cout << bind_func_member(100) << endl;;
   auto bind_func_member2 = bind(&OP::minus, op, placeholders::_1, 1000);
   cout << bind_func_member2(100) << endl;;
	return 0;
}

111
333
333
333
100
-100
800
900
-900

```


Since HTTP protocol is stateless itself, web application developed techniques to co create a session on top of HTTP so that servers could reconize multiple requetes from the same user as parts of a more complex and longer lasting squence (the user sessions)

**Managing HPPT Sessions**
1. Store session state in cookies: 
2.  Delegage the session storage to an external data store
3. Use a load balancer that supports sticky sessions

**Managing Files**
- Nomatter how you store your files, you should always try to use a Context Delivery Network (CDN) provider to deliver _public fiels_ to your end users. long expiration -> CDN cache the files forever. -> less traffic on orignal servers hosting these fiels.
for private content, file download requests will have to go to front-end web-application server, instead of CDN.
- S3 suppors the concept of private and public buckets.

**Managing other typies of state**
emample. multi server for ebay bidding: 
1. remove locking functionlity from the application code and create independent service from it.
2. user your new shared lock service on all of your web application servers to sharelocks globally. 
Zookeeper: districuted locking, application configuration management, leader election, and run-tie cluster membership information.

**Caching** 
CDN. 

**Auto -Scaling**  
Amazon:  EC2 O(elastic compute cloud) instance,  webserver machine image (AMI)

***Web Service***


**const variable. put const before/after datatype both work**
```cpp
int const x = 5;
const int x = 5;
```

**const pointer**
> when you declare a const reference, you're only making the data referred to const. References, by their very nature, cannot change what they refer to. Pointers, on the other hand, have two ways that you can use them: you can change the data pointed to, or change the pointer itself. Consequently, there are two ways of declaring a const pointer: one that prevents you from changing what is pointed to, and one that prevents you from changing the data pointed to.

_a pointer to constant data is nature_
```cpp
const int *p_int;
```
_the address store in the pointer itself is const: PUT const after the *_
```cpp
int x;
int * const p_int = &x;
```

**Binary search Smallest and Largest**

```cpp
vector<int> searchRange(vector<int>& nums, int target) {
      int left = 0, right = nums.size();
      int first = -1, second = -1;
      while (left < right){
        int mid = left + (right-left)/2;
        if (target == nums[mid]){
          first = mid;
          right = mid;
        }else if (nums[mid] > target) right = mid; // YL: not mid-1-> left < right && right is exclusive
        else left = mid+1;
      }   
      
      if (first == -1) return {-1,-1};
      cout << first << endl;
      second = first;
      left = first, right = nums.size();
      while (left < right){
        int mid = left + (right-left)/2;
        cout << mid << endl;
        if (target == nums[mid])
        {
          second = mid;
          left = mid+1;
         }else if (nums[mid] > target) right = mid;  // YL: not mid-1-> left < right && right is exclusive
         else left = mid+1;
       }
       return {first, second};
```



**find island DFS/BFS is not sufficient: use UNION_FIND ?**

> for (auto& m: M) erase(m.first) ---> doesn't work?


```cpp
/*
There are a total of n courses you have to take labelled from 0 to n - 1.

Some courses may have prerequisites, for example, if prerequisites[i] = [ai, bi] this means you must take the course bi before the course ai.

Given the total number of courses numCourses and a list of the prerequisite pairs, return the ordering of courses you should take to finish all courses.

If there are many valid answers, return any of them. If it is impossible to finish all courses, return an empty array.
*/

 vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites) {
      unordered_map<int, unordered_set<int>> asendant, desendant;
      for (auto& p: prerequisites){
        asendant[p[0]].insert(p[1]);
        desendant[p[1]].insert(p[0]);
      }
      
      vector<int> ret;
      for (int i=0; i<numCourses; i++){
        if (asendant.count(i) == 0){
          ret.push_back(i);
          if (desendant.count(i)){
            for (auto d: desendant[i])
              asendant[d].erase(i);
          }
        }
      }
      
      int valid = 1;
      while (valid){
        valid = 0;
        for (int i=0; i< numCourses; i++){   // YL. for (auto& a: asendatant)   fails;
          if (asendant.count(i) && asendant[i].size() == 0){
            valid = true;
            ret.push_back(i);
            if (desendant.count(i)){
              for (auto d: desendant[i]){
                cout << "desendant: " << d << endl;
                asendant[d].erase(i);
              }
            }  
            asendant.erase(i);
          }
        }
      }
      if (ret.size() != numCourses) return {};
      return ret;
    }


```



```cpp
/*
**Minimum Window Substring**

Solution
Given a string S and a string T, find the minimum window in S which will contain all the characters in T in complexity O(n).

Example:

Input: S = "ADOBECODEBANC", T = "ABC"
Output: "BANC"
Note:

If there is no such window in S that covers all characters in T, return the empty string "".
If there is such window, you are guaranteed that there will always be only one unique minimum window in S.
*/

 string minWindow(string s, string t) {
      unordered_map<char, int> M;
      for (auto c: t) M[c]--;
      auto count = M.size();
      int tail = 0, head = 0;
      string res = "";
      while (head < s.size()){
        auto c = s[head];
        if (!M.count(c)) {head++; continue;}
        if (++M[c] == 0) count--;
        if (count == 0){
          while (!M.count(s[tail]) || M[s[tail]] > 0){   //YL: M[s[tail]] > 0 not M[s[tail]] > 1
            if (M.count(s[tail])) M[s[tail]]--;
            tail++;
          }
          if (res.size() == 0 || res.size() > head-tail+1){
            res = s.substr(tail, head-tail+1); 
          }
        }
        head++;
      }
      return res;
    }
:camel: :camel: :camel:  

### Increasing Triplet Subsequence

```cpp
// DP O(n*n)
bool increasingTriplet(vector<int>& nums) {
   int len = nums.size();
   vector<int> dp(len, 1);
   for (int i=1; i<len; i++){
      for (int j=0; j<i; j++){
         if (nums[i] > nums[j]){
            dp[i] = max(dp[i], dp[j]+1);
            if (dp[i] > 2) return true;
         }
      }
   }
   return false;
}

// O(nlog(n))
bool increasingTriplet(vector<int>& nums) {
   vector<int> tmp;
   for (auto& n : nums){
      auto it = lower_bound(tmp.begin(), tmp.end(), n);
      if (it == tmp.end()){
         tmp.push_back(n);
         if (tmp.size() == 3) return true;
      }else{
         if (n<*it) *it = n;
      }
   }
   return false;
}
```

### Sliding window

**209 Minimum size subarray sum(all postive)**

```cpp
/* two pointer. for each index i, if sum[0..i]> reuired s. 
   move left pointer to right until < s
   only valid for ALL positive array.
*/
int minSubArrayLen(int s, vector<int>& nums) {
      int len = nums.size();
      if (len < 1 ) return 0;
      int left = 0, right = 0;
      int sum = 0, ret = INT_MAX;
      while (right < len){
        sum += nums[right];
        if (sum >=s){
          ret = min(ret, right-left+1);
          while (sum>=s){
            sum-=nums[left++];
            if (sum >=s) ret = min(ret, right-left+1);
          }
        }
        right++;
      }
      return ret == INT_MAX ? 0 : ret;
    }
```

**862 Minimum size subarray sum(allow negative)**
```cpp
/*

Difference from "ALL_POSITIVE array" in 209  is to 
mantain an increasing list which remove all previous nodes 
which is larger than myself. 

*/
int shortestSubarray(std::vector<int>& A, int K) {
  int len = A.size();
  if (len == 0) return -1;
  if (A[0] >= K) return 1;
  deque<int> D;
  int ret = INT_MAX;
  for (int i = 0; i< len; i++ ){
    if (i>0) A[i]+=A[i-1];
    if (A[i] >=K ) ret = min(ret, i+1);
    while (D.size() && A[i] - A[D.front()] >= K) {
      ret = min(ret, i-D.front());
      D.pop_front();
    }
    /* all previous A[node] on D is in increasing older. */
    /* check skyline problem and see if same idea applies*/
    while (D.size() && A[i] < A[D.back()]){
       D.pop_back();
    }
    D.push_back(i);
  }  
  return ret == INT_MAX ? -1:ret;

}
```
[solution](https://leetcode.com/problems/shortest-subarray-with-sum-at-least-k/discuss/143726/C%2B%2BJavaPython-O(N)-Using-Deque)


## sliding window problem

* 1248
* 1234 
* 1004
* 930
* 992
* 904
* 862 
* 209 
## To do:  all question for in order

### 94. Binary tree inorder traversal

```cpp
/* iteration */
vector<int> inorderTraversal(TreeNode* root) {
      vector<int> ret;
      stack<TreeNode*> S;
      while (S.size() || root){
        if (root){
          S.push(root);
          root = root->left;
        }else{
          auto top = S.top();
          ret.push_back(top->val);
          S.pop();
          root = top->right;
        }
      }
      return ret;
}

// to do: Morris traversal method

```


### 106. Construct Binary Tree from Inorder and Postorder Traversal

```cpp
/*
Given inorder and postorder traversal of a tree, construct the binary tree.

Note:
You may assume that duplicates do not exist in the tree.

For example, given

inorder = [9,3,15,20,7]
postorder = [9,15,7,20,3]

Return the following binary tree:

    3
   / \
  9  20
    /  \
   15   7
*/

    unordered_map<int,int> M;
    TreeNode* helper(vector<int>& inorder, vector<int>& postorder, int& idx, int left, int right) {
      if (left > right) return NULL;
      TreeNode* root = new TreeNode(postorder[idx]);
      auto left_idx = M[postorder[idx]]-1;
      auto right_idx = M[postorder[idx]]+1;
      idx--;
      root->right = helper(inorder, postorder, idx, right_idx, right);
      root->left = helper(inorder, postorder, idx, left, left_idx);
      return root;
    }
  
    TreeNode* buildTree(vector<int>& inorder, vector<int>& postorder) {
      int len = inorder.size(), idx = 0;
      for (auto& i: inorder) M[i]=idx++;
      idx = len-1;
      return helper(inorder, postorder, idx, 0, len-1);
    }
```
### 105. Construct Binary Tree from Inorder and preorder Traversal (same idea)


### 285. Inorder Successor in BST

```cpp
   TreeNode* helper(TreeNode* root, TreeNode* large, TreeNode* p){
       if (root == p){
         if (root->right){
           auto tmp = root->right;
           while (tmp && tmp->left){
             tmp = tmp->left;
           }
           return tmp;
         }else if (large) { 
            return large; 
         }else{
           return nullptr;
         }
       }else if (root->val > p->val){
         if (root->left){
           return helper(root->left, root, p);
         }else{
           return nullptr;
         }
       }else{
         if (root->right){
           return helper(root->right, large, p);
          }else{
           return nullptr;
         }
       }    
    }
  
    TreeNode* inorderSuccessor(TreeNode* root, TreeNode* p) {
      return helper(root, nullptr, p);
    }
```




[Hight lectins food;](https://riordanclinic.org/2017/11/lectins-food-inflamed-tired/)

Food High in Lectins:

	*	Wheat
	*	Corn
	*	Rice
	*	White potatoes
	*	Oats
	*	Quinoa
	*	Legumes (including peanuts and cashews)
	*	Soy
	*	Seeds (pumpkin, sunflower, chia)
	*	Cucumbers
	*	Zucchini
	*	Pumpkins
	*	Squashes (of any kind)
	*	Tomatoes
	*	Eggplant
	*	eppers
	*	Goji berries


[Rao Institute](https://www.youtube.com/watch?v=uLDz7emCnZI)

	*	Control uncontroable (first world war)
	*	Mental chatting (worry too much)
	*	Process vs Goal (stay on Mount everest)
	*	Departing forever (No guarantee reunion)


[Source: find substring using KMP](kmp_subStrIndex.cpp)
 
**to do:**
* 	understand how to build KMP lps array 
* 	understand how to use   KMP lps array

[KMP explaination](http://jakeboxer.com/blog/2009/12/13/the-knuth-morris-pratt-algorithm-in-my-own-words/)

[LC28 solution using KMP](https://leetcode.com/problems/implement-strstr/discuss/12956/C%2B%2B-Brute-Force-and-KMP)

[git OZturk from OSU](https://github.com/ozturkosu)
