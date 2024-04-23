```cpp
vim block  insert   (exc twice)
1. select  block area with  ctr-v
2. shift i or  I
3 insert context
4. exc TIWCE 


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
You use dynamic_cast to cast a pointer/RERENCE from a base class to a derived class if you have polymorphic classes (virutal desctructior is needed) and want to perform a safe downcast.

For casting from a subclass to its superclass, you can use a regular cast (static_cast) because it's always a valid conversion.

static_cast is a more general casting. pointer, refernce, object, built-in. It's the programmer's responsibility to make that the casting is valid. So you can blindly replace all dynamic_cast with static_cast in the codebase without introduing build issure during compile time. But it is preferred to use dynmaic_cast + success/failure(null) check + optional try/catch block to dispatch logic.


In this example, Base is a polymorphic class with a virtual function, and Derived is derived from Base. We create a Derived object and assign its address to a Base*. Later, we use dynamic_cast to safely downcast the Base* to a Derived*. If the object is indeed of the derived type, the dynamic_cast will succeed, and we can safely call functions specific to the derived class using the resulting pointer (derivedPtr). If the cast fails (for example, if the object is not of the derived type), dynamic_cast returns a null pointer.

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


```
> It's M[s[tail]] > 0 instead of M[s[tail]] > 1


:herb: :herb: [Binary search tree problem tempalte from leetcode discussion](https://leetcode.com/discuss/general-discussion/786126/python-powerful-ultimate-binary-search-template-solved-many-problems)

:herb: :herb: [**lower_bound/upper_bound similar login in vector and map**](https://leetcode.com/explore/interview/card/google/67/sql-2/3045/)
>  find the first element >=k    it = M. lower_bound(k) && it != M.end()    *it     >=  k  \
>  find the first element <=k    it = M.upper_bound(k) && it != M.begin()   *(--it) <=  K 

:herb: :herb: [fb os](https://docs.google.com/drawings/d/1ci7Iq0XvNYICmBBzS9go_9KiRMYPuUw7gdq3H9A8zv0/edit?ts=5f15dafe)

:herb: :herb: [fb profile](https://www.facebookrecruiting.com/portal/home)

:herb: :herb: [3 sum. how to remove duplicated answer](https://leetcode.com/problems/3sum/submissions/)
```cpp
vector<vector<int>> threeSum(vector<int>& nums) {
        vector<vector<int>> ret = {};
        if (nums.size() < 3) return ret;
        sort(nums.begin(), nums.end());
        for (int i=0; i<nums.size()-2;i++){
          int target = -nums[i];
          int start = i+1, end  = nums.size()-1; // outer loop: always shift. What if 4 sum? 
          while (start < end){
            auto val = nums[start] + nums[end];
            if (val == target) {
              ret.push_back({nums[i], nums[start], nums[end]});
              while (start < end && nums[start+1] == nums[start]) start++;  //inner loop: shift only when answer found.
              while (start < end && nums[end] == nums[end-1]) end--;        //inner loop: shift only when answer found.
              star t++;end--;
            }else if (val < target){
              start++;
            }else{
              end--;
            }
          }
        }
        return ret;
    }

```

:herb: :herb: [merge two BST]("https://leetcode.com/problems/all-elements-in-two-binary-search-trees/")
```cpp
  vector<int> getAllElements(TreeNode* root1, TreeNode* root2) {
     stack<TreeNode*> s1, s2;
     while (root1){
       s1.push(root1);
       root1=root1->left;
     }
     while (root2){
       s2.push(root2);
       root2=root2->left;
     }
      
     vector<int> ret;
     while (s1.size() || s2.size()){
       if (!s1.size() || (s2.size() && s2.top()->val <= s1.top()->val)){ // <======= YL. combine if statements.
         auto tmp = s2.top(); s2.pop();
         ret.push_back(tmp->val);
         auto tmp2 = tmp->right;
         while (tmp2){
           s2.push(tmp2);
           tmp2 = tmp2->left;
         }
       }else{
         auto tmp = s1.top(); s1.pop();
         ret.push_back(tmp->val);
         auto tmp2 = tmp->right;
         while (tmp2){
           s1.push(tmp2);
           tmp2 = tmp2->left;
         }
       }
     }
     return ret;
    }

```

:herb: :herb: [ map<int,vector<set>>> for vertical tree print](https://leetcode.com/problems/vertical-order-traversal-of-a-binary-tree/)
```cpp
void traverse(TreeNode* root, int x, int y, map<int,vector<set<int>>>& M){
      if (!root) return;
      if (!M.count(x)) M[x] = vector<set<int>>();
      if (M[x].size() < y+1){
        int diff = y-M[x].size()+1;
        while (diff--) M[x].push_back(set<int>());
      }
      M[x][y].insert(root->val);
      traverse(root->left, x-1, y+1, M);
      traverse(root->right, x+1, y+1, M);
    }
  
    vector<vector<int>> verticalTraversal(TreeNode* root) {
      map<int, vector<set<int>>> M;
      traverse(root, 0, 0, M);
      vector<vector<int>> ret;
      for (auto& m: M){
        vector<int> tmp;
        for (auto& v: m.second){
          tmp.insert(tmp.end(), v.begin(), v.end());
        }
        if (tmp.size()) ret.push_back(tmp);
      }
      return ret;
    }
  

```

:herb: :herb: [Simply unix path](https://leetcode.com/problems/simplify-path/submissions/)
>   use find("/").   substr could be "." or ""
```cpp
 string simplifyPath(string path) {
      int start = 0;
      deque<string> DQ;
      while (start <path.size()){
        string s1;
        auto pos = path.find("/", start);
        if (pos == string::npos){
          s1 = path.substr(start);
          start = path.size();
        }else{
          s1 = path.substr(start, pos-start);
          start = pos+1;
        }
        if (s1 == "." || s1=="") {
          //do nothing
        }else if (s1 == ".."){
          if (DQ.size()) DQ.pop_front();
        }else{
          DQ.push_front(s1);
        }
      }
      string ret = "/";
      while (DQ.size()){
        ret += DQ.back(); DQ.pop_back();
        if (DQ.size()) ret +="/";
      }
      return ret;
    }
  
```

:herb: :herb: [find range for target value: lower_Bound. !!!!!](https://leetcode.com/problems/find-first-and-last-position-of-element-in-sorted-array/submissions/)
```cpp
 int lowerBound(vector<int>& nums, int target){
      int l = 0, r = nums.size()-1;
      while (l <= r){
        auto m = l + (r-l)/2;
        if (nums[m] < target)
          l = m+1;
        else            // <=========  eles inlcue both "=="" and ">"
          r = m-1;
      }
      return l;
    }
  
    vector<int> searchRange(vector<int>& nums, int target) {
      vector<int> ret = {-1,-1};
      auto l = lowerBound(nums, target);
      if (l >= nums.size() || nums[l] != target) return ret;
      ret[0] = l;
      ret[1] = lowerBound(nums, target+1)-1;
      return ret;
    }

```

:herb: :herb: [search in rotated array. 1. find smallest idx. binary search two array;](https://leetcode.com/explore/interview/card/facebook/54/sorting-and-searching-3/279/)
```cpp
 int smallestIdx(vector<int>& nums, int start, int end){
      if (start == end) return start;
      while (start < end){
        int mid = (start+end)/2;
        if (nums[mid] > nums[mid+1]) return mid+1; //YL: mid+1 would OVERFLOW !!!!!
        if (nums[mid] > nums[end]){
          start = mid+1;
        }else{
          end = mid;    //YL <================== not mid-1
        }
      }   
      return start;
    }
    
    int binarySearch(vector<int>& nums, int target, int start, int end){
      while (start <= end) {
        int mid = start+(end-start)/2;
        if (nums[mid] == target) return mid;
        if (nums[mid] < target){
          start = mid+1;
        }else {
          end = mid-1;
        }
      }
      return -1;
    }
  
    int search(vector<int>& nums, int target) {
      if (nums.size() == 0) return -1;
      int idx = smallestIdx(nums, 0, nums.size()-1);
      if (nums[idx] == target) return idx;
      if (nums[idx] > target) return -1;
      if (nums.back()>=target)
        return binarySearch(nums, target, idx+1, nums.size()-1);
      else
        return binarySearch(nums, target, 0, idx-1);
    }

```
:herb: :herb: [divided be careful about INT_MAX, and base*2 > INT_MAX](https://leetcode.com/explore/interview/card/facebook/54/sorting-and-searching-3/308/)
```cpp
  int divide(int dividend, int divisor) {
      int sign = 1;
      if ((dividend >0 && divisor < 0) || (dividend <0 && divisor > 0)) {
        sign = -1;
      }
      
      unsigned d1, d2;
      if (dividend == INT_MIN){
        if (divisor == -1) return INT_MAX; 
        if (divisor == 1) return INT_MIN;
        d1 = abs(INT_MIN+1); d1++;
      }else d1 = abs(dividend);
      
      if (divisor == INT_MIN){
        if (dividend == INT_MIN) return 1; 
        if (d1 < INT_MAX) return 0;
        d2 = abs(INT_MIN+1); d2++;
      }else d2 = abs(divisor);
      
      if (d1 < d2) return 0;
      
      int ret = 0;
      while (d1 >= d2){
         unsigned base = d2;
         int count = 1; 
         while (base <=INT_MAX/2  && base*2 < d1) {
           count*=2;
           base*=2;
         }
         ret+=count;
         d1 -= base;
      }
      return sign*ret;
    }

```

:herb: :herb: [pertubeII vs pertubeI:  sort. skip when used[cur-1] && val[cur-1]==val[cur] ](https://leetcode.com/explore/interview/card/facebook/53/recursion-3/293/)
```cpp
  void bt(vector<int>& V, vector<vector<int>>& ret, vector<int>& nums, vector<bool>& used, int idx){
      if (idx == nums.size()) {
        ret.push_back(V); return;
      }
      for (int i=0; i<nums.size(); i++){
        if (!used[i]){
          if (i!=0 && used[i-1] && nums[i]==nums[i-1]) continue;  ;//YL  for removing  duplicate.
          V.push_back(nums[i]);
          used[i] = true;
          bt(V, ret, nums, used, idx+1);
          used[i] = false;
          V.pop_back();
        }
      }
    }
  
    vector<vector<int>> permuteUnique(vector<int>& nums) {
      sort(nums.begin(), nums.end());  // for removing duplicate
      vector<vector<int>> ret;
      vector<bool> used(nums.size(),false);
      vector<int> V;
      bt (V, ret, nums, used, 0);
      return ret;
    }
```

:herb: :herb: [Convert BST to double linked list](https://leetcode.com/explore/interview/card/facebook/52/trees-and-graphs/544/)
```cpp
  pair<Node*, Node*> helper(Node* root)
    {
      if (!root) return {nullptr, nullptr};
      auto left = helper(root->left);
      if (left.second) {
        left.second->right = root;
        root->left = left.second;
      }
      auto right = helper(root->right);
      if (right.first){
        root->right = right.first;
        right.first->left = root;
      }
      return {left.first? left.first : root, right.second?right.second:root};
    }
  
    Node* treeToDoublyList(Node* root) {
      if (!root) return root;
      auto ret = helper(root);
      ret.first->left = ret.second;
      ret.second->right = ret.first;
      return ret.first;
    }
```



:herb: :herb:
[flatten tree to single linkedlist](https://leetcode.com/explore/interview/card/facebook/52/trees-and-graphs/322/)
```cpp
 void flatten(TreeNode* root) {
      TreeNode** root_p = &root;
      while (root){
        if (root->left){
          auto tmp = root->left; 
          while (tmp && tmp->right){
            tmp = tmp->right;
          }
          tmp->right = root->right;
          root->right = root->left; 
          root->left = nullptr;
          root = root->right;
        }else{
          root = root->right;
        }
      }
      root = *root_p;
    }
```



:herb: :herb:
[linkedlist reverse, fast_slow pointer, reorderList](https://leetcode.com/explore/interview/card/facebook/6/linked-list/3021/)
```cpp
 ListNode* reverse(ListNode* head)
   {
      ListNode dummy(0);
      ListNode* pre = &dummy;
      pre->next = head;
      while (head && head->next){
        ListNode* head_next = head->next;
        ListNode* pre_next = pre->next;
        if (head_next) head->next = head_next->next;
        head_next->next = pre_next;
        pre->next = head_next;
      }
      return dummy.next;
    }
  
    ListNode* reorderList(ListNode* head)
    {
      
      if (!head || !head->next) return head;
      ListNode* fast = head;
      ListNode* slow = head;
      while (fast && fast->next){
        fast = fast->next->next;
        slow = slow->next;
      }
      auto right = slow->next;
      slow->next = nullptr;
      right = reverse(right);
      
      ListNode** head_p = &head;
      while (right){
        auto head_next = head->next;
        auto right_next = right->next;
        head->next = right;
        right->next = head_next;
        head = head_next;
        right = right_next;
      }
      return *head_p;
    }

```

:herb: :herb: [doc from mitbbs](https://www.zero1code.info/)

:herb: :herb: [system design notes by chiXu](http://chixu.me/interview/2018/02/19/System-Design) &nbsp; &nbsp; &nbsp; [system design link mitbbs 2014](https://www.mitbbs.com/article_t/JobHunting/32777529.html)

:herb: :herb: [FB interview tools coderpad/google draw/sysmtem disign github](https://leetcode.com/discuss/interview-experience/687776/nda-facebook-e5-menlo-park-jun-2020-offer)

:herb: :her: [FB recent coding test combination](https://leetcode.com/discuss/general-discussion/675445/facebook-interview-experiences-all-combined-from-lc-till-date-07-jun-2020)

[K's element in sorted matrix](https://leetcode.com/submissions/detail/349023136/)
> 1. priority queue K*lg(min(K,#of row/col)\ 
> 2. binary seearch. YL.   pair<cur_left, cur_right> in while loop.  Return left, instead of arbitary.

[divide two intergers. YL](https://leetcode.com/submissions/detail/346804822/)

[3SUM--> remove duplicated inner loop and outer loop](https://leetcode.com/submissions/detail/345729529/)\
[backtracer questions](https://leetcode.com/problems/subsets/discuss/27281/A-general-approach-to-backtracking-questions-in-Java-(Subsets-Permutations-Combination-Sum-Palindrome-Partitioning))

[FBO from LC with tips](https://leetcode.com/discuss/interview-experience/637356/amazon-apple-facebook-l5-ict4-e5-seattle-april-2020-may-2020-offer-offer-offer)
## 05/18 ##
## [monotonous stack](https://leetcode.com/problems/smallest-subsequence-of-distinct-characters/discuss/308210/JavaC%2B%2BPython-Stack-Solution-O(N)) ##

[linkedlist: split + reverse + merge ](https://leetcode.com/problems/reorder-list/discuss/45003/A-concise-O(n)-time-O(1)-in-place-solution)

```cpp
/*
Given a singly linked list L: L0→L1→…→Ln-1→Ln,
reorder it to: L0→Ln→L1→Ln-1→L2→Ln-2→…

You may not modify the values in the list's nodes, only nodes itself may be changed.

Example 1:

Given 1->2->3->4, reorder it to 1->4->2->3.
Example 2:

Given 1->2->3->4->5, reorder it to 1->5->2->4->3.
*/

     // helper:  revert linked-list
    ListNode* reverse(ListNode* head){
       if (!head) return head;
       ListNode dummy(0);     
       ListNode* pre = &dummy;
       dummy.next = head;
       while (head && head->next){
         auto tmp = pre->next;
         pre->next = head->next;
         head->next = head->next->next;
         pre->next->next = tmp;
       }
       return dummy.next;
    }
  
    ListNode* reorderList(ListNode* head){
      
       if (!head || !head->next)  return head;
      
       // split list: slow is at left(middle) 
       // slow->next is the begin of right (size of len/2, could 1 less than left half)
       ListNode *fast = head, *slow = head;
       while (fast && fast->next){
         slow = slow->next;
         fast = fast->next->next;
       }
       
       ListNode* right = slow->next;
       right = reverse(right);
       slow->next = nullptr;
       
      // merge two list 
      auto new_head = head;
       while (right){
         auto right_next = right->next;
         auto tmp = new_head->next;
         new_head->next = right;
         right->next = tmp;
         new_head = new_head->next->next;
         right = right_next;
       }
       return head;
    }

```

> This problem should be solved as if there are exact same characters in both arraies. 

```cpp
/*
Reverse to Make Equal
Given two arrays A and B of length N, determine if there is a way to make A equal to B by reversing any subarrays from array B any number of times. 
Example
A = [1, 2, 3, 4]
B = [1, 4, 3, 2]
output = true
After reversing the subarray of B from indices 1 to 3, array B will equal array A.
*/


```
### 958
> BFS queue . layer by layer. Once found a null. then all children from that point should be null
```cpp

 bool isCompleteTree(TreeNode* root) {
        if (!root) return true;
        queue<TreeNode*> Q;
        bool found_null = false;
        Q.push(root);
        while (Q.size()){
            auto front = Q.front(); Q.pop();
            if (front->left) {
                if (found_null) return false;
                Q.push(front->left);
            }else{
                found_null = true; 
            }
            if (front->right) {
                if (found_null) return false;
                Q.push(front->right);
            }else{
                found_null = true; 
            }
        }
        return true;
    }

```



#### ciper problem.
> using tempoary factor to avoid being over-written
> 'a-z', 'A->Z" using %20. '1-->0' using %10
> **be careful** c-'a'+ (factor%26)  instead of (c-'a'+factor)%26. 

> ciper. be careful about 
```cpp
string rotationalCipher(string input, int rotationFactor) {
  string ret; 
  for (auto c: input){
    auto rotationFactor_new = rotationFactor; // YL. need temporary factor
    if (c>='a' && c<='z'){
        c -= 'a';
        rotationFactor_new %= 26;  // in case factor is too large. module **first**
        c+=rotationFactor_new;
        c%=26;
        c+='a';
    }else  if (c>='A' && c<='Z'){
        c -= 'A';
        rotationFactor_new %= 26;
        c+=rotationFactor_new;
        c%=26;
        c+='A';
    }else  if (c>='0' && c<='9'){
        c -= '0';
        rotationFactor_new %= 10;
        c+=rotationFactor_new;
        c%=10;
        c+='0';
    }
    ret.push_back(c);
  }
  return ret;
  
}


```



> (N) and N(lgK) method for **K Closest Points to Origin** problem
```cpp

/* priority queue O(NlgK)*/

  struct comp {
        bool operator()(vector<int>& A, vector<int>& B){
            return A[0]*A[0] + A[1]*A[1] < B[0]*B[0] + B[1]*B[1];
        }
    };
    
    vector<vector<int>> kClosest(vector<vector<int>>& points, int K) {
        vector<vector<int>> ret;
        priority_queue<vector<int>, vector<vector<int>>, comp> PQ;
        for (auto& p: points){
            PQ.push(p);
            if (PQ.size() > K) PQ.pop();
        }
        while (PQ.size()){
            ret.push_back(PQ.top()); PQ.pop();
        }
        return ret;
    }



/*quick sort* O(N) */

 int quickSort(vector<vector<int>>& points, int start, int end){
        int x = points[end][0], y=points[end][1];
        int left = start, right = end-1;
        while (left <= right){
            if ( (points[left][0]*points[left][0]+points[left][1]*points[left][1]) <= x*x+y*y){
                left++;
            }else if ( (points[right][0]*points[right][0]+points[right][1]*points[right][1]) > x*x+y*y){
                right--;
            }else{
                swap(points[left], points[right]);
            }
        }
        swap(points[left], points[end]);
        return left;
    }
    
    vector<vector<int>> kClosest(vector<vector<int>>& points, int K) {
        int start =0, end = points.size()-1;
        int idx = quickSort(points, 0, end);
        while (idx != K-1){
            if (idx > K-1){
                end = idx-1;
            }else{
                start = idx+1;
            }
            idx = quickSort(points, start, end);
        }
        vector<vector<int>> ret;
        for (int i=0; i<K; i++) ret.push_back(points[i]);
        return ret; 
    }

```


```cpp
/*
Given an array of integers and an integer k, you need to find the total number of continuous subarrays whose sum equals to k.

Example 1:
Input:nums = [1,1,1], k = 2
Output: 2
Note:
The length of the array is in range [1, 20,000].
The range of numbers in the array is [-1000, 1000] and the range of the integer k is [-1e7, 1e7].
*/
  int subarraySum(vector<int>& nums, int k) {
        int ret = 0,  sum = 0;
        unordered_map<int, int> M;
        M[k]=1;  //this is begining, when first sum==k, ret++;
        for (auto n : nums){
            sum += n;
            ret += M[sum];
            //from this n, sum+k is expected. but previous location may also expect sum+k
            //M[sum+k]++, instead of M[sum+k] = 1
            M[sum+k]++; 
        }
        return  ret;
    }
```


[**MIT 6006** Nick Whites suggestion ](https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-006-introduction-to-algorithms-fall-2011/lecture-videos/)

[**Pramp** Nick Whites suggestion ](https://www.hackerrank.com/interview/interview-preparation-kit)

[**hackerrank** Nick Whites suggestion ](https://www.hackerrank.com/interview/interview-preparation-kit)


[**numbers with Equal Digit Sum**]("https://leetcode.com/discuss/interview-question/365872/")
> skip. 

[**Min Moves to Obtain String Without 3 Identical Consecutive Letters**](https://leetcode.com/discuss/interview-question/398026/)
> ** To do. no clue question. any pattern for the solution**

[**Max Network Rank** ](https://leetcode.com/discuss/interview-question/364760/)
> Ambiguity question?  need read fast and comprehend correctly

[**Min Adj Swaps to Make Palindrome**](https://leetcode.com/discuss/interview-question/351783/)
> 



> 1235. Maximum Profit in Job Scheduling \


```cpp
/*
We have n jobs, where every job is scheduled to be done from startTime[i] to endTime[i], obtaining a profit of profit[i].

You're given the startTime , endTime and profit arrays, you need to output the maximum profit you can take such that there are no 2 jobs in the subset with overlapping time range.

If you choose a job that ends at time X you will be able to start another job that starts at time X.

*/

 struct job
    {
       int start, end, profit;
       job(int s, int e, int p):start(s), end(e), profit(p){};
       bool operator<(const job& other) const {
           if (end < other.end) return true;
           return false;
       }
        
    };
    int jobScheduling(vector<int>& startTime, vector<int>& endTime, vector<int>& profit) {
       
       int len = startTime.size(); 
       if (len < 1) return 0;
       vector<job> V;
       for (int i=0; i<startTime.size(); i++)
           V.push_back(job(startTime[i],endTime[i], profit[i]));
       sort(V.begin(), V.end());
       vector<int> DP(len);
       DP[0] = V[0].profit;
       for (int i=1; i<len; i++){
           DP[i] = max(DP[i-1], V[i].profit);
           job tmp(0, V[i].start, 0);
           // Seems binary search(192ms) is slower than search backword one by one (152)
           bool use_upper_bound = 0;
           if (use_upper_bound){ // binary search
               auto it = upper_bound(V.begin(),V.end(),tmp);
               if (it-V.begin()>0) DP[i] = max(DP[i], DP[it-V.begin()-1]+V[i].profit);
           }else{ // search backward one by one
               for (int j=i-1; j>=0; j--){
                   if (V[j].end <= V[i].start){
                       DP[i] = max(DP[i], DP[j]+V[i].profit);
                       break; //YL break as no need to compare left items. 
                   }
               }
           } 
       }
       return DP[len-1];                                     
    }
```


> 2D water trapping\
> using priority_queue \
> Pay attention to struct T. 

```cpp

 struct T {
      int x,y,h;
      T(int a, int b, int c):x(a),y(b),h(c){};
      bool operator<(const T& other) const {return h>other.h;}
    };
  
    int trapRainWater(vector<vector<int>>& heightMap) {
       int row = heightMap.size();
       if (!row) return 0;
       int col = heightMap[0].size();
       priority_queue<T> pq;
      
       vector<vector<int>> visited(row, vector<int>(col,0));
       for (int i=0; i<col; i++){
         pq.push(T(0,i,heightMap[0][i]));
         visited[0][i] = 1;
         if (row > 1) {
           pq.push(T(row-1,i,heightMap[row-1][i]));
           visited[row-1][i] = 1;
         }
       }
      
      // four corners are duplicated. But it's OK.
       for (int i=0; i<row; i++){
         pq.push(T(i,0,heightMap[i][0]));
         visited[i][0] =1;
         if (col > 1){
           pq.push(T(i,col-1,heightMap[i][col-1]));
           visited[i][col-1] =1;
         }
       }
       
       int ret = 0;
       vector<vector<int>> dir = {{0,1}, {0,-1}, {-1,0}, {1,0}};
       while (pq.size()){
         auto top = pq.top(); pq.pop();
         for (auto& d: dir){
           int x = top.x + d[0];
           int y = top.y + d[1];
           if (x<0 || x==row || y<0 || y==col || visited[x][y]) continue;
           auto tmp = max(0, top.h - heightMap[x][y]);
           ret+= tmp;
           visited[x][y] = 1;
           pq.push(T(x,y,max(heightMap[x][y], top.h))); // YL. h is max(top.h, this_high)
         }
       }
       return ret;
    }

```


> 778. Swim in Rising Water \
> On an N x N grid, each square grid[i][j] represents the elevation at that point (i,j). \
> Now rain starts to fall. At time t, the depth of the water everywhere is t. \
> You can swim from a square to another 4-directionally adjacent square if and only if the elevation of both squares individually are at most t. \
> You can swim infinite distance in zero time. Of course, you must stay within the boundaries of the grid during your swim. \
> You start at the top left square (0, 0). What is the least time until you can reach the bottom right square (N-1, N-1)?



```cpp

  struct T
    {
      int h, x, y;
      T(int hh, int xx, int yy) : x(xx), y(yy), h(hh) {};
      bool operator< (const T &d) const {return h > d.h;}
      //bool operator<(T &t)  {return h > t.h;}  // without 'const' pq.push fails
    };
  
  
    int swimInWater(vector<vector<int>>& grid) {
       int N = grid.size();
       vector<vector<int>> dir = {{0,1},{0,-1}, {-1,0}, {1,0}};
       priority_queue<T> pq;
       vector<vector<int>> visited(N,vector<int>(N,0));
       visited[0][0] = 1;
       pq.push(T(grid[0][0],0 ,0));
       int max_level=grid[0][0];
       while (pq.size()){
         auto top = pq.top();
         pq.pop();
         max_level = max(max_level, top.h);
         if (top.x==N-1 && top.y==N-1) return max_level;
         for (auto d: dir){
           int x = top.x + d[0];
           int y = top.y + d[1];
           if (x >=0 && x < N  && y >=0 && y<N && !visited[x][y]){
             visited[x][y] = 1;
             pq.push(T(grid[x][y], x, y));
           }
         }
       }
       return -1;
    }

```


> swap linkedlist in pair
> Given 1->2->3->4, you should return the list as 2->1->4->3.\
> **always** make real change from **root->next = blah blah**, **not root = blahblah**\
> checkout other's soluton which are much elegent

```cpp
 ** Others 1**
 ListNode* swapPairs(ListNode* head) {
      if(head == NULL || head->next == NULL) return head; 
      ListNode *cur, *nxt, *nhead;
      
      nhead = NULL;
      cur = head;
      nxt = cur->next;
      cur->next = nxt->next;
      nxt->next = cur;
      cur->next = swapPairs(cur->next);
      if(nhead == NULL) nhead = nxt;
      
      return nhead; 
    }
};

ListNode* swapPairsIterative(ListNode* head) {
        ListNode tmp{0};
        tmp.next = head;
        ListNode *pre = &tmp, *cur = head;
        while(cur != NULL && cur->next != NULL) {
            pre->next = cur->next;
            pre = pre->next;
            cur->next = pre->next;
            pre->next = cur;
            cur = cur->next;
            pre = pre->next;
        }
        return tmp.next;
    }
    
    ListNode* swapPairsRecursive(ListNode* p1) {
        if (p1 == NULL || p1->next == NULL) {
            return p1;
        }
        ListNode* res = p1->next;
        ListNode* np = swapPairs(p1->next->next);
        p1->next->next = p1;
        p1->next = np;
        return res;
    }

**mine**
 void advance(ListNode* dummy, ListNode* root){
       if (!root || !root->next){
          dummy->next = root;
          return;
       }
       auto next = root->next->next;
       dummy->next = root->next;
       dummy->next->next = root;
       root->next = next;
    }
  
    ListNode* swapPairs(ListNode* head) {
      ListNode dummy(0);
      auto root = &dummy;
      while (head){
         advance(root, head); 
         if (root->next && root->next->next){
           root = root->next->next;
           head = root->next;
         }else{
           break;
         }
      }
      return dummy.next;
    }



```


> LFU
> three map. 

```cpp
class LFUCache {
public:
  
    unordered_map<int, pair<int, int>> key_val_freq; 
    unordered_map<int, list<int>> freq_list;
    unordered_map<int, list<int>::iterator> key_iter; 
    int _capacity;
    int min_freq;
    
    LFUCache(int capacity) {
      _capacity = capacity;     
    }
    
    int get(int key) {
      if (_capacity <= 0) return -1;
      if (!key_val_freq.count(key)) return -1;
      auto pair = key_val_freq[key];
      auto val = pair.first;
      auto cur_freq = pair.second;
      freq_list[cur_freq].erase(key_iter[key]);
      freq_list[cur_freq+1].push_front(key);
      key_iter[key]= freq_list[cur_freq+1].begin();
      if (freq_list[cur_freq].size() == 0 && min_freq==cur_freq){
        min_freq++;
      }
      key_val_freq[key].second++;
      return val;   
    }
    
    void put(int key, int value) {
        if (_capacity <= 0) return;
        auto id = get(key);
        if (id !=-1){
          key_val_freq[key].first = value;
          return;
        }
        if (key_val_freq.size() == _capacity){
          auto key_to_remove = freq_list[min_freq].back();
          freq_list[min_freq].pop_back();
          key_iter.erase(key_to_remove);
          key_val_freq.erase(key_to_remove);
        }
        min_freq = 1;
        freq_list[min_freq].push_front(key);
        key_iter[key] = freq_list[min_freq].begin();
        key_val_freq[key] = {value, min_freq};
    }
};

```

> LRU

```cpp
class LRUCache {
public:
  
    using L = list<pair<int, int>>;
    using M = unordered_map<int, L::iterator>;
  
    LRUCache(int capacity) {
        _capacity = capacity;
    }
    
    int get(int key) {
      if (m.count(key)){
        auto it = m[key];
        l.splice(l.begin(), l, it);
        m[key] = l.begin();
        return it->second;
      }
      return -1;
    }
    
    void put(int key, int value) {
      if (get(key) != -1){
        m[key]->second = value;
        return;
      }
      if (m.size() == _capacity){
        auto back = l.back();
        auto key = back.first;
        l.pop_back();
        m.erase(key);
      }
      l.push_front({key, value});
      m[key] = l.begin();
    }
    
private:
    int _capacity;
    L l;
    M m;
  
};

```

> word ladder II
> online soluton uses DFS, check-it-out

```cpp
vector<string> getNeighbor(string& s, set<string>& ss, set<string>& visited){
       int len = s.size();
       vector<string> ret;
       for (int i= 0; i<len; i++){
         int c = s[i] - 'a';
         for (int j=0; j<26; j++){
           if (j== c) continue;
           s[i] = 'a' + j;
           if (ss.count(s) && !visited.count(s)) 
             ret.push_back(s); 
         }
         s[i] = 'a' + c; 
       }
       return ret; 
    }
  
    vector<vector<string>> findLadders(string beginWord, string endWord, vector<string>& wordList) {  
      
      unordered_map<string, vector<string>> parent;
      set<string> visited;
      set<string> wordS;
      for (auto w: wordList) wordS.insert(w);
      
      queue<string> Q;
      Q.push(beginWord);
      
      vector<vector<string>> ret;
      bool terminate = false;
      
      while (!terminate && Q.size()){
        auto len = Q.size(); 
        set<string> tmp;
        for (int i=0; i<len; i++){
          visited.insert(beginWord);
          auto top = Q.front(); Q.pop();
          auto neighbors = getNeighbor(top, wordS, visited);
          for (auto n: neighbors){
            if (n==endWord) terminate = true;
            parent[n].push_back(top);
            tmp.insert(n);
          }
        }
        for (auto& t: tmp) {Q.push(t); visited.insert(t);}
      }
      for (auto p: parent[endWord])
        cout << p << " ";
      
      // endword is unreachable from startword   
      if (!parent.count(endWord)) return ret;
      queue<vector<string>> QQ;
      QQ.push({endWord});
      while (QQ.size()){
        int len = QQ.size();
        for (int i=0; i<len; i++){
          auto top = QQ.front();
          QQ.pop();
          auto word = top[0];
          for (auto p: parent[word]){
            auto item = top;
            item.insert(item.begin(), p);
            if (p == beginWord) ret.push_back(item);
            else QQ.push(item);
          }
        }
      }
      return ret;
    }
```

> recover binary search tree\
> in helper, if a pointer is changed, use reference: last_visited, first, second.\
> root is OK as it is not changed in the program. \

```cpp
   
   void helper(TreeNode*& last_visited, TreeNode* root, TreeNode*& first, TreeNode*& second){
      if (!root) return;
      auto val = root->val;
      helper(last_visited, root->left, first, second);
      if (val < last_visited->val){
        if (first == nullptr) {first = last_visited; second = root;}
        else {second = root;} 
      }
      last_visited = root;
      helper(last_visited, root->right, first, second);
    }
    void recoverTree(TreeNode* root) {
      if (!root) return;
      TreeNode dummy(INT_MIN);
      auto last = &dummy;
      TreeNode *first = nullptr, *second = nullptr; 
      helper(last, root, first, second);
      swap(first->val, second->val);
    }

```


Dijistra runtime E(lgV),  space: E+V

> 992. Subarrays with K Different Integers

```cpp
992. Subarrays with K Different Integers 
Given an array A of positive integers, call a (contiguous, not necessarily distinct) subarray of A good if the number of different integers in that subarray is exactly K.
(For example, [1,2,3,1,2] has 3 different integers: 1, 2, and 3.)
Return the number of good subarrays of A.
Example 1:
Input: A = [1,2,1,2,3], K = 2
Output: 7
Explanation: Subarrays formed with exactly 2 different integers: [1,2], [2,1], [1,2], [2,3], [1,2,1], [2,1,2], [1,2,1,2].

int maxSubarrayAtMostKDistinct_suffix(vector<int>& A, int K) {
     int ret = 0, left=0, len = A.size();
     unordered_map<int, int> M;
     for (int i=0; i<A.size(); i++){
        if (!M[A[i]]++) K--;
        while (K<0){
           if (!--M[A[left++]]) K++;
        } 
        ret+=i-left+1;
     }   
     return ret;
  }

int subarraysWithKDistinct(vector<int>& A, int K) {
    return maxSubarrayAtMostKDistinct_suffix(A, K) -
          maxSubarrayAtMostKDistinct_suffix(A, K-1);
} 

```


> spatial matrix
```cpp
/*
59. Spiral Matrix II
Given a positive integer n, generate a square matrix filled with elements from 1 to n2 in spiral order.

Example:

Input: 3
Output:
[
 [ 1, 2, 3 ],
 [ 8, 9, 4 ],
 [ 7, 6, 5 ]
]
*/

#include <vector>
#include "header.h"

int main(){
    int n =3;
    vector<vector<int>> dir = {
      {0,1},  // east
      {1,0},  // south
      {0,-1}, //west
      {-1,0}   //north
    };
    vector<vector<int>> V(n, vector<int>(n,0));
    int total = n*n;
    int count = 1;
    int r = 0, c = 0, d = 0;
    while (count <= total){
       cout << count << endl;
       V[r][c] = count++;
       if (count > total) break;
       int new_row = r+dir[d][0];
       int new_col = c+dir[d][1];
       while (new_row<0 || new_row==n || new_col <0 || new_col == n || V[new_row][new_col] != 0){
          d = (d+1)%4;
          new_row = r+dir[d][0];
          new_col = c+dir[d][1];
       }
       r= new_row, c = new_col;
    }
    printVV(V);
    return 0;
}

```

> search sorted 2D matrix
```cpp
 bool searchMatrix(vector<vector<int>>& matrix, int target) {
      int rows = matrix.size();
      if (rows == 0 ) return false;
      int cols = matrix[0].size();
      int row = rows-1, col = 0;
      while(row >=0 && col < cols){
        if (matrix[row][col] == target) return true;
        else if (matrix[row][col] > target){
          row--;
        }else{
          col++;
        }
        //cout << "row: " << row << " col: " << col << endl;
      }
      return false;
    }
```

>subtring of another tree
```cpp
 bool isSame(TreeNode* s, TreeNode* t){
      if (!s && !t ) return true;
      if (!s || !t) return false;
      return (s->val == t->val && isSame(s->left, t->left) && isSame(s->right, t->right));
    }
  
    bool isSubtree(TreeNode* s, TreeNode* t) {
      if (isSame(s, t)) return true;
      if (s && isSubtree(s->left, t)) return true;
      if (s && isSubtree(s->right, t)) return true;
      return false;
    }
```


> merge two sorted list (EASY)
```cpp
 ListNode* mergeTwoLists(ListNode* l1, ListNode* l2) {
      ListNode root(0);
      ListNode* cur = &root;
      while(l1 || l2){
        if (!l1){
          cur->next = l2;
          l2=l2->next;
        }else if (!l2){
          cur->next = l1; 
          l1=l1->next;
        }else{
          if (l1->val < l2->val){
            cur->next = l1;
            l1=l1->next;
          }else{
            cur->next = l2;
            l2=l2->next;
          }
        }
        cur = cur->next;
      }
      return root.next;
    }
```

> Copy random list (hashing)
```cpp
 Node* copyRandomList(Node* head) {
      
      if (!head) return NULL;
      
      unordered_map<Node*, Node*> M;
      Node* new_head = new Node(0, NULL, NULL);
      
      Node *cur_old = head, *cur_new = new_head;
      while (cur_old){
        cur_new->val = cur_old->val;
        M[cur_old] = cur_new;
        cur_old = cur_old->next;
        if (cur_old){
          cur_new->next = new Node(0, NULL, NULL);
          cur_new = cur_new->next;
        }
      }
      
      cur_old = head; cur_new = new_head;
      while (cur_old){
        cur_new->random = M[cur_old->random];;
        cur_old = cur_old->next;
        cur_new = cur_new->next;
      }
      
      return new_head;
        
    }

```

> treasureIsland II
```cpp

#include <queue>
#include <iostream>
#include <unordered_set> 
#include <set>

using namespace std;

int findTreasure(vector<vector<char>>& M){
   int row = M.size(), col = M[0].size(), res = 0;
   vector<vector<int>> dir = {{0,-1},{0,1},{-1,0},{1,0}};
   queue<vector<int>> Q;
   for (int i=0; i<row; i++){
     for (int j=0; j<col; j++){
        if (M[i][j] == 'S') {
           M[i][j] = 'D';
           Q.push({i,j});
        }
     }
   }

   while(Q.size()){
     int len = Q.size();
     for (int k=0; k<len; k++){
       auto s = Q.front(); Q.pop();
       int i=s[0], j=s[1]; 
       for (auto d: dir){
          int ii=i+d[0], jj=j+d[1];
          if (ii<0 || ii == row || jj<0 || jj == col || M[ii][jj] =='D') continue;
          if (M[ii][jj] == 'O'){
             Q.push({ii,jj});
             M[ii][jj] = 'D';
          }else if (M[ii][jj] == 'X'){
             return ++res;
          }
       }
     }
     res++;
   }
   return -1;
}

int main(){

vector<vector<char>> input = 
   {{'S', 'O', 'O', 'S', 'S'},
   {'D', 'O', 'D', 'O', 'D'},
   {'O', 'O', 'O', 'O', 'X'},
   {'X', 'D', 'D', 'O', 'O'},
   {'X', 'D', 'D', 'D', 'O'}};

  int count = findTreasure(input);
  cout << count << endl; 
  return 0;
}


```


> treasureIsland BFS DFS
```cpp

#include <iostream>
#include <vector>
#include <queue>
#include <utility>

using namespace std;

// BFS
int minStepsBFS(vector<vector<char>>& island) {
    int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    int m = island.size(), n = island[0].size();

    queue<pair<int, int>> q;
    q.push(std::pair<int, int>(0, 0)); island[0][0] = 'D';

    int steps = 1;
    while (!q.empty()) {
        int levelSize = q.size();
        for (int i = 0; i < levelSize; ++i) {
            auto pos = q.front();
            q.pop();
            for (auto dir: dirs) {
                int nx = pos.first + dir[0];
                int ny = pos.second + dir[1];
                if (nx < 0 || nx >= m || ny < 0 || ny >= n || island[nx][ny] == 'D') continue;
                if (island[nx][ny] == 'X') return steps;
                q.push(pair<int,int>(nx, ny));
                island[nx][ny] = 'D';
            } 
        }
        
        ++steps;
    }

    return -1;
}

// DFS
int res = INT_MAX;

void dfs(vector<vector<char>>& island, const vector<vector<int>>& dirs, int x, int y, int steps) { 
    if (x < 0 || x >= island.size() || y < 0 || y >= island[0].size() || island[x][y] == 'D' || steps > res) return;
    if (island[x][y] == 'X') {
        res = std::min(res, steps);
        return;
    }

    island[x][y] = 'D';
    for (auto dir: dirs) {
        int nx = x + dir[0];
        int ny = y + dir[1];
        dfs(island, dirs, nx, ny, steps+1);
    }
}

int minStepsDFS(vector<vector<char>>& island) {
    vector<vector<int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    dfs(island, dirs, 0, 0, 0);
    return res == INT_MAX ? -1 : res;
}

int main() {
    vector<vector<char>> island = {{'O', 'O', 'O', 'O', 'O'},
                                   {'D', 'D', 'D', 'O', 'O'},
                                   {'O', 'O', 'O', 'O', 'D'}, 
                                   {'O', 'D', 'X', 'D', 'O'}};

    vector<vector<char>> islandcopy(island);
    cout << minStepsBFS(island) << endl;
    cout << minStepsDFS(islandcopy) << endl;
}

```

>favoriteGenres

```cpp
#include <unordered_map>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include "header.h"

using namespace std;

unordered_map<string, vector<string>> favoriteGenres(
  unordered_map<string, vector<string>>& M1, 
  unordered_map<string, vector<string>>& M2)
{

  unordered_map<string, string> M3;

  for (auto m2: M2){
    for(auto s: m2.second) M3[s]=m2.first;
  }


  unordered_map<string, vector<string>> ret;
  for (auto m1: M1){
     int _max = 0;
     unordered_map<string, int> M4;
     for (auto& s: m1.second){
        auto type = M3[s];
	M4[type]++;
	_max = max(_max, M4[type]);
     }
     vector<string> ret0;
     for (auto tmp: M4){
	if (tmp.second == _max) ret0.push_back(tmp.first);
     }
     ret[m1.first]=ret0;
  }
  return ret;
}

int main(){

   unordered_map<string, vector<string>> M1 =  {

      {"David", {"song1", "song2", "song3", "song4", "song8"}},
      {"Emma",  {"song5", "song6", "song7"}}
   };

   unordered_map<string, vector<string>> M2 =  {

      {"Rock", {"song1", "song3"}},
      {"Dubstep", {"song7"}},
      {"Techno", {"song2", "song4"}},
      {"Pop", {"song5", "song6"}},
      {"Jazz", {"song8", "song9"}},

   };
   auto A = favoriteGenres(M1, M2);
   for (auto a: A){
      cout << a.first << endl;
      printV(a.second);
   }
   return 0;
}

```

> connect rope/sticks

```cpp
 int connectSticks(vector<int>& sticks) {
      priority_queue<int, vector<int>, greater<int>> PQ;
      int ret = 0;
      for (auto s : sticks) PQ.push(s);
      while (PQ.size() > 1){
         auto top1 = PQ.top(); PQ.pop();
         auto top2 = PQ.top(); PQ.pop();
         auto val = top1+top2;
         ret+=val;
         PQ.push(val);
      }
      return ret;
    }

```

> Pair closest to target

```cpp
#include <vector>
#include <algorithm>
#include <climits>
#include "header.h"

using namespace std;

vector<vector<int>> closePair(vector<vector<int>>& A, vector<vector<int>>& B, int target){

   auto comp = [&](vector<int>&A, vector<int>& B){return A[1]<B[1];};
   sort(A.begin(), A.end(), comp);
   sort(B.begin(), B.end(), comp);

   int left=0, right = B.size()-1;
   int diff = INT_MAX;
   vector<vector<int>> ret;

   while (left<A.size() && right>=0){
      auto cur_diff = target - (A[left][1]+B[right][1]); 
      if (cur_diff < 0 || cur_diff > diff){
        right--;
      }else{
        if (cur_diff < diff){
          diff = cur_diff;
          ret.clear();
        }
        ret.push_back({A[left][0], B[right][0]});
        left++;
      }
   }
   return ret;
}

int main(){
  vector<vector<int>> A={{1, 3}, {2, 5}, {3, 7}, {4, 10}};
	vector<vector<int>> B=  {{1, 2}, {2, 3}, {3, 4}, {4, 5} };
  auto ret = closePair(A,B,10);
  for (auto r: ret) printV(r);
  return 0;
	//Output: [[2, 4}, [3, 2}*
} 

```

> 373. Find K Pairs with Smallest Sums 
```cpp
/*
373. Find K Pairs with Smallest Sums 
You are given two integer arrays nums1 and nums2 sorted in ascending order and an integer k. 
Define a pair (u,v) which consists of one element from the first array and one element from the second array. 
Find the k pairs (u1,v1),(u2,v2) ...(uk,vk) with the smallest sums. 
Example 1: 
Input: nums1 = [1,7,11], nums2 = [2,4,6], k = 3
Output: [[1,2],[1,4],[1,6]] 
Explanation: The first 3 pairs are returned from the sequence: 
             [1,2],[1,4],[1,6],[7,2],[7,4],[11,2],[7,6],[11,4],[11,6]  
             
*/

 struct comp{
    bool operator()(pair<int,int>& A, pair<int,int>& B){
      return A.first+A.second > B.first+B.second;
    }
  };
  
public:
    vector<vector<int>> kSmallestPairs(vector<int>& nums1, vector<int>& nums2, int k) {
       if (nums1.empty() || nums2.empty()) return {};
       auto comp = [&nums1, &nums2](pair<int, int>&A, pair<int,int>&B){
         return nums1[A.first]+nums2[A.second] > nums1[B.first] + nums2[B.second];};
       priority_queue<pair<int, int>, vector<pair<int,int>>, decltype(comp)> PQ(comp);
       PQ.push({0,0});
       vector<vector<int>> ret;
       while (PQ.size() && k--){
         auto t = PQ.top(); PQ.pop();
         ret.push_back({nums1[t.first], nums2[t.second]});
         if (t.first<nums1.size()-1)
           PQ.push({t.first+1, t.second});
         if (t.first == 0 && t.second < nums2.size() -1 )
           PQ.push({t.first, t.second+1});
       }
       return ret;
    }

```




> brige and articaultion node.\
> bridge v->w\
> Method 2: In any DFS search tree, a tree edge v->w is a bridges if and only if there is no back egde from w(child) ---> v(parent).\
> method 1: for any DFS search tree, of v->w is bridge if w[first] == w[second] && w is not 0 (sing there is no -1->0 edge).\
  
> to do:  articaulation node
> N need at least two egde
> N.first < N[neibor].second

```cpp
 // bridge v->w
  // Method 2: In any DFS search tree, a tree edge v->w is a bridges if and only if there is no
  // back egde from w(child) ---> v(parent).
  
  // method 1: for any DFS search tree, of v->w is bridge if w[first] == w[second] && w is not 0 (since  there is no -1->0 edge).
  
  // articulaton node:
  // N has at leat two neighbor
  // N.first <= any N[neighor].second N is aritcualtion node.
  
  vector<vector<int>> ret; 
  void helper(int idx, int parent, unordered_map<int, set<int>>& link, unordered_map<int, pair<int,int>>& score){
      int level = score.size();
      score[idx].first = level;
      score[idx].second = level;
      for (auto i: link[idx]){
        if (i == parent) continue; 
        if (!score.count(i)) helper(i, idx, link, score); 
        score[idx].second = min(score[idx].second, score[i].second);  
      } 
      // method 1
      if (score[idx].second == score[idx].first && idx != 0) ret.push_back({idx, parent}); 
    }
  
  
    vector<vector<int>> criticalConnections(int n, vector<vector<int>>& connections) { 
      unordered_map<int, set<int>> link;
      unordered_map<int, pair<int,int>> score; 
      for (auto& c: connections) {link[c[0]].insert(c[1]); link[c[1]].insert(c[0]);};
      helper(0, -1, link, score);  
     /* method 2
      for (auto c: connections){
        int tail = score[c[0]].first < score[c[1]].first ? c[0] : c[1];
        int head = tail == c[0] ? c[1]: c[0]; 
        if (score[head].second > score[tail].first) ret.push_back(c); 
      }
     */
      
      return ret;
    }
    }
```




> Max average sub-tree
```cpp
 pair<int, double> average(TreeNode* root, double& max_average){
      if (!root) return {0, 0.0};
      auto left = average(root->left, max_average );
      auto right = average(root->right, max_average);
      int count = left.first + right.first +1;
      int sum = left.second + right.second + root->val;
      double average = (double)sum/count;
      max_average = max(max_average, average);
      return {count, sum};
    }
    double maximumAverageSubtree(TreeNode* root) {
      double max_average = numeric_limits<double>::lowest();
      average(root, max_average);
      return max_average;
}
```



> connectRope
```cpp
  int connectSticks(vector<int>& sticks) {
      priority_queue<int, vector<int>, greater<int>> PQ;
      int ret = 0;
      for (auto s : sticks) PQ.push(s);
      while (PQ.size() > 1){
         auto top1 = PQ.top(); PQ.pop();
         auto top2 = PQ.top(); PQ.pop();
         auto val = top1+top2;
         ret+=val;
         PQ.push(val);
      }
      return ret;
}

```

> recordered log
```cpp
 vector<string> reorderLogFiles(vector<string>& logs) {
       vector<string> ret;
       int count = 0;
       for (auto& log: logs){
         auto loc = log.find(" ");
         if (isdigit(log[loc+1])) ret.push_back(log);
         else{
           ret.insert(ret.begin(), log);
           count++;
         }
       }
       sort(ret.begin(), ret.begin()+count, [&](string& A, string& B){
         auto loc1 = A.find(" "); auto loc2 = B.find(" ");
         return (A.substr(loc1) < B.substr(loc2));
       });
            ;
      return ret;
}

```

> topToys
```cpp
/*
input:
int numToys = 6
int topToys = 2
vector<string> toys={"elmo", "elsa", "legos", "drone", "tablet", "wacraft"};
vector<string> quotes = {
"Elmo is the hotest of the season! Elmo will be on every kid's wishlist!",
"The new Elmo dolls are supper high quality",
"Expect the Elsa dolls to be very popular this year, Elsa!", 
"Elsa and Elmo are the toys I'll be buying for my kids, Elsa is good", 
"For parents of older kids, look into buying them a drone", 
"Warcraft is slowly rising in popularity ahead of the holiday season"};
output: 
{"elmo", "elsa"};
*/


#include <string>
#include <unordered_map>
#include <set>
#include <queue>
#include "header.h"

using namespace std;

struct comp{
   bool operator()(pair<string, int>& A, pair<string, int>& B ){
      if (A.second < B.second) return true;
      if (A.second > B.second) return false;
      return A.first > B.first;
   }
};



vector<string> getToken2(string& s){
   vector<string> ret;
   size_t left=0, right=0, len = s.size();
   while (right < len){
      while (!isalpha(s[left])) left++;
      while (!isalpha(s[right])) right++;
      while ( isalpha(s[right]) && right<len) right++;
      if (left < len) ret.push_back(s.substr(left, right-left));
      left=right;
   } 
   return ret;
}

vector<string> getToken(string& s){
   vector<string> ret;
   size_t pos =0, new_pos;
   while((new_pos=s.find(" ",pos))!=string::npos){
      auto A = s.substr(pos, new_pos-pos);
      transform(A.begin(), A.end(), A.begin(), [](char c){return tolower(c);});
      ret.push_back(A);
      pos=new_pos+1;
   }
   ret.push_back(s.substr(pos));
   printV(ret);
   return ret;
}


vector<string> getTop(vector<string>& quotes, vector<string> toys, int top){
   set<string> S;
   unordered_map<string, int> M;
   priority_queue<pair<string, int>, vector<pair<string,int>>, comp> PQ;
   for (auto& t: toys) S.insert(t);
   for (auto& q : quotes){
      auto V = getToken(q);
      auto V2 = getToken2(q);
      printV(V); printV(V2);
      set<string> ss;
      for (auto& v: V){
      	if (S.count(v)) ss.insert(v);
      }
      for (auto& a: ss) M[a]++;
   }
   for (auto& m: M) PQ.push({m.first, m.second});
   vector<string> ret;
   while (top-- && PQ.size()) {
     ret.push_back(PQ.top().first);
     PQ.pop();
   }
   return ret;
}

int main(){

  vector<string> toys={"elmo", "elsa", "legos", "drone", "tablet", "wacraft"};
  vector<string> quotes = {
	"Elmo is the hotest of the season! Elmo will be on every kid's wishlist!",
	"The new Elmo dolls are supper high quality",
	"Expect the Elsa dolls to be very popular this year, Elsa!", 
	"Elsa and Elmo are the toys I'll be buying for my kids, Elsa is good", 
	"For parents of older kids, look into buying them a drone", 
	"Warcraft is slowly rising in popularity ahead of the holiday season"
  };

  auto ret = getTop(quotes, toys, 2);
  printV(ret);
  return 0;
}

```

```cpp
/*
1268. Search Suggestions System 
Given an array of strings products and a string searchWord. We want to design a system that suggests at most three product names\
from products after each character of searchWord is typed. Suggested products should have common prefix with the searchWord. \
If there are more than three products with a common prefix return the three lexicographically minimums products.\

Return list of lists of the suggested products after each character of searchWord is typed. 
Example 1:

Input: products = ["mobile","mouse","moneypot","monitor","mousepad"], searchWord = "mouse"
Output: [
["mobile","moneypot","monitor"],
["mobile","moneypot","monitor"],
["mouse","mousepad"],
["mouse","mousepad"],
["mouse","mousepad"]
]
Explanation: products sorted lexicographically = ["mobile","moneypot","monitor","mouse","mousepad"]
After typing m and mo all products match and we show user ["mobile","moneypot","monitor"]
After typing mou, mous and mouse the system suggests ["mouse","mousepad"]
*/

 struct trie {
      vector<trie*> child = vector<trie*>(26, nullptr);
      bool isWord = false;     
    };
  
    void addWord(trie* t, string& word){
      for (auto c: word){
        if (t->child[c-'a'] == nullptr) t->child[c-'a'] = new trie();
        t = t->child[c-'a'];
      }
      t->isWord = true;
    }
  
    trie* find(trie* t, string sub){
      if (t == nullptr) return t;
      for (auto c: sub){
        if (t->child[c-'a'] == nullptr) return nullptr;
        else t = t->child[c-'a'];
      }
      return t;
    }
  
    void extract2(trie* t, string& s,  vector<string>& S){
      if (S.size() == 3 || t == nullptr) return;
      if (t->isWord) S.push_back(s);
      for (int i=0; i<26 && S.size() < 3; i++){
        if (t->child[i] != nullptr) {
           s.push_back('a' + i);
           extract2(t->child[i], s, S);
           s.pop_back(); 
        }
      }
    }
  
    vector<vector<string>> suggestedProducts(vector<string>& products, string searchWord) {
      trie* root = new trie();
      for (auto w: products) addWord(root, w);
      vector<vector<string>> ret;
      for (int i=1; i<=searchWord.size(); i++){
        vector<string> s;
        auto str = searchWord.substr(0,i);
        auto t = find(root, str);
        extract2(t, str, s);
        ret.push_back(s);
      }
      return ret;
    }
```



> rotten orange

```cpp
 int orangesRotting(vector<vector<int>>& grid) {
       queue<pair<int,int>> Q;
       int day = 0;
       vector<pair<int,int>> dir = {{-1,0},{1,0}, {0,-1}, {0,1}};
       for (int i=0; i<grid.size(); i++){
         for (int j=0; j<grid[0].size(); j++){          
           if (grid[i][j] == 2) Q.push({i,j});
         }
       }
      
       while (Q.size()){
          int len = Q.size();
          for (int i=0; i<len; i++){
            auto top = Q.front(); Q.pop();
            int row = top.first, col = top.second;
            for (auto d: dir){
              int i=row+d.first, j=col+d.second;
              if (i>=0 && i<grid.size() && j>=0 && j<grid[0].size() && grid[i][j] == 1){
                Q.push({i,j});
                grid[i][j] = 2;
              }
            }
          }
          if (Q.size()) day++;
       }
      
       for (auto& row : grid){
         for (auto& col: row)
           if (col == 1) return -1;
       }
       return day;
    }
```

> number of island (BFS)
```cpp
int numIslands(vector<vector<char>>& grid) {
      queue<pair<int,int>> Q;
      vector<pair<int,int>> dir = {{-1,0}, {1,0}, {0,-1},{0,1}};
      int count = 0;
      for (int i=0; i<grid.size(); i++){
        for (int j=0; j<grid[0].size(); j++){
          if (grid[i][j] == '1'){
            count++;
            Q.push({i,j}); grid[i][j] = '2';
            while(Q.size()){
               auto top = Q.front(); Q.pop(); 
               int i2 = top.first, j2 = top.second;
               for (auto d: dir){
                 int ii=i2+d.first, jj=j2+d.second;
                 if (ii<0 || ii==grid.size() || jj<0 || jj==grid[0].size() ||  grid[ii][jj] != '1')
                     continue;
                 Q.push({ii,jj}); grid[ii][jj] = '2';
               }
              }
            }
         }
      }
      return count;
   }

```


> 132 PATTERN. **monotonic stack** \
> if stack.top(x) < small[j-1]. then it is useless because it must smaller than all small[j-x] 

```cpp
 bool find132pattern(vector<int>& nums) {
      size_t len = nums.size();
      if (len < 3) return false;
      auto small = nums;
      for (int i=1; i<len; i++) small[i]=min(nums[i], small[i-1]);
      stack<int> S;
      for (int j= len-1; j>0; j--){
        while (S.size() && S.top() < nums[j]){
          if (small[j-1]<S.top())
            return true;
          else
            S.pop();
        }
        S.push(nums[j]);
      }
      return false;
    }
```


> 254 factor combination.  recurive. 
> be care for to avoid dumplication by using left limit. 
> need revisit the logic

```cpp
public:
    vector<vector<int>> ret;
    void helper(int n, int left, vector<int> pre){
        for (int i=left; i<=sqrt(n); i++){
          if (n%i==0) {
             auto V = pre; 
             V.push_back(i);
             helper(n/i, i, V);
          }
        }
        if (pre.size()){
          pre.push_back(n);
          ret.push_back(pre);
        }
    }
    vector<vector<int>> getFactors(int n) {
      vector<int> pre;
      helper(n,2, pre);
      return ret;
    }
```

> priority_queue comp lamba
[lc solution](https://leetcode.com/problems/find-k-pairs-with-smallest-sums/discuss/84607/Clean-16ms-C%2B%2B-O(N)-Space-O(KlogN)-Time-Solution-using-Priority-queue)

```cpp
/* 
You are given two integer arrays nums1 and nums2 sorted in ascending order and an integer k.
Define a pair (u,v) which consists of one element from the first array and one element from the second array.
Find the k pairs (u1,v1),(u2,v2) ...(uk,vk) with the smallest sums.
*/
```

> ## Increasing/decreasing stack.
> Don't dereferece vector without initilize its size.

```cpp
/*
739. Daily Temperatures
Given a list of daily temperatures T, return a list such that, for each day in the input, 
tells you how many days you would have to wait until a warmer temperature. 
If there is no future day for which this is possible, put 0 instead.
For example, given the list of temperatures 
T = [73, 74, 75, 71, 69, 72, 76, 73], your output should be 
    [1, 1, 4, 2, 1, 1, 0, 0].
*/  
    
   vector<int> dailyTemperatures(vector<int>& T) {
      vector<int> ret(T.size(), 0);  // YL;  program crash without initilization ret.
      stack<int> S;
      for (size_t i=0; i < T.size(); i++){
        int val = T[i];
        while (S.size() && val > T[S.top()]){
          int idx = S.top();
          ret[idx] = i-idx;
          S.pop();
        }
        S.push(i);
      }
      return ret;
    }  
    
```


[DP summary](https://leetcode.com/discuss/general-discussion/458695/dynamic-programming-patterns)


> pointer pass by refernce **TreeNode*& newRoot**;

```cpp
/*
156. Binary Tree Upside Down
Given a binary tree where all the right nodes are either leaf nodes with a sibling (a left node that shares the same parent node) or empty, flip it upside down and turn it into a tree where the original right nodes turned into left leaf nodes. Return the new root.

Example:
Input: [1,2,3,4,5]
    1
   / \
  2   3
 / \
4   5

Output: return the root of the binary tree [4,5,2,#,#,3,1]
   4
  / \
 5   2
    / \
   3   1  
*/
TreeNode* helper(TreeNode* root, TreeNode*& newRoot) {  //YL: pointer pass by reference
      if (!root) return root;
      auto left = root->left;
      auto right = root->right;
      if (left) {
         auto flip = helper(left, newRoot);
         flip->left = right;
         flip->right = root;
         root->left = nullptr;
         root->right = nullptr;
         return root;
      }else{
        if (!newRoot) newRoot = root;
        return root;
      }
    }
    TreeNode* upsideDownBinaryTree(TreeNode* root) {
      TreeNode* newRoot = nullptr;
      helper(root, newRoot);
      return newRoot;
    }

```
> use lambda comparison function in lower_bound.  neither comp function nor comp functor works\
> be careful how to use (lower_bound-1) and (upper_bound)\
> what happens when start and end iterator are the same in erase? do nothing and return the start/end +1 ? \

```cpp
/*
57. Insert Interval
Given a set of non-overlapping intervals, insert a new interval into the intervals (merge if necessary).
You may assume that the intervals were initially sorted according to their start times.
Example 1:
Input: intervals = [[1,3],[6,9]], newInterval = [2,5]
Output: [[1,5],[6,9]]

Example 2:
Input: intervals = [[1,2],[3,5],[6,7],[8,10],[12,16]], newInterval = [4,8]
Output: [[1,2],[3,10],[12,16]]
Explanation: Because the new interval [4,8] overlaps with [3,5],[6,7],[8,10].
*/
    // use lambda function in lower_bound/upper_bound
    vector<vector<int>> insert(vector<vector<int>>& intervals, vector<int>& newInterval) {
      auto comp3 = [&](vector<int> A, vector<int> B){return A[0] < B[0];};
      auto comp4 = [&](vector<int> A, vector<int> B){return A[1] < B[1];};
      vector<vector<int>>::iterator left = lower_bound(intervals.begin(), intervals.end(), newInterval, comp3);
      auto right = upper_bound(intervals.begin(), intervals.end(), newInterval, comp4);
      if (left != intervals.begin() &&  (*(left-1))[1] >= newInterval[0]){  // YL: left-1
          left--;
          newInterval[0] = (*left)[0];
      }
      if (right != intervals.end() && newInterval[1] >= (*right)[0]){ //YL: right itself
          newInterval[1] = (*right)[1];
          right++;
      }
      auto it = intervals.erase(left, right);
      intervals.insert(it, newInterval);
      return intervals;
    }
  
    // comparison function doesn't work with lower_bound 
    static bool comp1(vector<int>& A, vector<int>& B){
         return A[0]  < B[0];
      }
    // comparison functior doesn't work with lower_bound 
    struct comp2{
      bool operator()(vector<int>& A, vector<int>& B){
        return A[0] < B[0];
      }
    };
```


### 01/29
> 2sum question. 
> remember [...-nums[i]

```cpp
/*
given an array of integers and an integer k, you need to find the total number of continuous subarrays whose sum equals to k.

Example
Input:nums = [1,1,1], k = 2
Output: 2

Note:

    The length of the array is in range [1, 20,000].
    The range of numbers in the array is [-1000, 1000] and the range of the integer k is [-1e7, 1e7].
*/

 int subarraySum(vector<int>& nums, int k) {
      unordered_map<int, int> M;
      int sum = 0, ret = 0;
      for (int i=0; i<nums.size(); i++){
        if (nums[i] == k) ret++;
        sum+=nums[i];
        if (M.count(sum)) ret+= M[sum];
        M[sum+k-nums[i]]++;  // YL M[sum+k-nums[i]], not M[sum+k]
      }
      return ret;
    }

```



### 01/26

[Minimum diffficult of a Job schedule/ split array largest sum/ palindrome partitioning III](https://leetcode.com/problems/minimum-difficulty-of-a-job-schedule/discuss/490297/Java-Bottom-Up-DP)

[SL](https://www.quanwei.tech/?q=digimarc&job=engineer)

### 01/22

#### Sort square of sorted array

> lower_bound: Returns an iterator pointing to the first element in the range [first,last) which compares **no less than** than val.\
> upper_bound: Returns an iterator pointing to the first element in the range [first,last) which compares **greater** than val. \
> [1,2,4] -> lower_bound(3) and upper_bound(3) should be the same: *it = 4. 
> erase(iterator A [, iterator B ]) : return  iterator pointing to the new location of the element that followed the last element erased by the function call. \
> This is the container end if the operation erased the last element in the sequence.
```cpp
  vector<int> sortedSquares(vector<int>& A) {
      auto it = upper_bound(A.begin(), A.end(), 0);  //YL use upper_bound instead of linear search. 
      int pos = it - A.begin();
      int neg = pos - 1, len = A.size();
      vector<int> ret;
      while (neg >= 0 || pos < len ){ // YL use len instead of A.size() since size() would be called in each iteration
        if (neg >=0 && pos < len){
          auto neg_square = A[neg]*A[neg];
          auto pos_square = A[pos]*A[pos];
          if (neg_square <= pos_square){
            ret.push_back(neg_square); neg--;
          }else{
            ret.push_back(pos_square); pos++;
          }
        }else if (neg >= 0){
            auto neg_square = A[neg]*A[neg];
            ret.push_back(neg_square); neg--;
        }else{
          auto pos_square = A[pos]*A[pos];
          ret.push_back(pos_square); pos++;
        }
      }
      return ret;
    }

```

### 01/21
#### 3sum closest
[-1, -1, 1, 1, 3] target -1

> to reduce redudant \
> the last two while loop should be: \
> left > i+i (not i !) to make sure that at least the second -1 is not skip
> right < nums.size()-1, to avoid overflow

```cpp
    int threeSumClosest(vector<int>& nums, int target) {
    sort(nums.begin(), nums.end());
    long ret = INT_MAX;
    for (int i=0; i<nums.size()-2; i++){
      if (i && i<nums.size()-2 && nums[i]==nums[i-1]) continue;
      int left = i+1, right = nums.size()-1;
      while (left < right){
        long val = nums[i] + nums[left] + nums[right];
        if (abs(val -target) < abs(ret-target)) ret = val;
        if (val < target) left++;
        else right--;
        while (left > i+i && left < right && nums[left] == nums[left-1]) left++; // YL   left > i+1
        while (left < right && right < nums.size()-1 && nums[right] == nums[right+1]) right--; // YL  right < nums.size()-1
      }
    }
      return ret;
    }

```

### 01/20
> sort linkedlist\
> merge function return both start/end node\
> bottom up approach. 

```cpp
pair<listNode*, listNode*> merge(listNode* a, listNode* b){
   listNode dummy(0);
   listNode* cur = &dummy;
   while (a || b){
      if (a && b){
         if (a->val < b->val){
            cur->next = a;
            a = a->next;
         }else{
            cur->next = b;
            b = b->next;
         }
      }else if (a){
         cur->next = a;
         break;
      }else{
         cur->next = b;
         break;
      }
      cur = cur->next;
   }
   while (cur && cur->next) cur = cur->next; //YL. don't forget advancing cur to the last node.
   return {dummy.next, cur};
}

listNode* ll_sort(listNode* root){
   int len = 0;
   auto root2 = root;
   while (root2){len++; root2=root2->next;}
   listNode dummy(0);
   dummy.next = root;
   for (int step=1; step < len; step*=2){
      cout << "step: " << step << endl;
      auto cur = &dummy;
      while (cur && cur->next){

         auto left = cur->next;
         auto left2 = left;
         int count = step;
         while (left2 && --count > 0 ) {left2 = left2->next;}
         auto right = left2?left2->next:NULL;
         if (left2) left2->next = NULL;

         count = step;
         auto right2 = right;
         while (--count > 0 && right2 && right2->next) {right2 = right2->next;}
         auto next_start = right2?right2->next:NULL;
         if (right2) right2->next = NULL;

         auto lr = merge(left, right);
         cur->next = lr.first;
         cur = lr.second;
         cur->next = next_start;
      }
      dummy.next->print();
   }
   return dummy.next;
}

```



## We are what we read, watch, eat.
## Man's power

### 01/18
**quick sort, merge sort (recursive and bottom up)**
```cpp
int quick_sort_partitian(vector<int>& V, int left, int right){
  int right_save = right;
  int val = V[right_save];
  while (left <= right){
    if (V[left] < val) left++;
    else if (V[right]>=val) right--;
    else swap(V[left++], V[right--]);
  }
  swap(V[left], V[right_save]);
  return left;
}

void _quickSortHelper(vector<int>& nums, int start, int end){
   if (start >= end) return;
   int paviot = quick_sort_partitian(nums, start, end);
   _quickSortHelper(nums, start, paviot-1);
   _quickSortHelper(nums, paviot+1, end);
}

void quickSort(vector<int>& nums){
   _quickSortHelper(nums,0,nums.size()-1);
}

void merge(vector<int>& V, int start, int mid, int end){
   vector<int> tmp;
   int left = start, right = mid+1;
   while (left <=mid || right <= end){
      if (left <=mid && right <= end){
         if (V[left] < V[right])
            tmp.push_back(V[left++]);
         else
            tmp.push_back(V[right++]);
      }else if (left<=mid){
         tmp.push_back(V[left++]);
      }else{
         tmp.push_back(V[right++]);
      }   
   }   
   for (int i=0; i<=end-start; i++) V[start+i] = tmp[i];
}
void mergeSort_topDown_helper(vector<int>& V, int start, int end){
  if (start >= end) return;  //YL.
  int mid = start + (end - start)/2;
  mergeSort_topDown_helper(V, start, mid);
  mergeSort_topDown_helper(V, mid+1, end);
  merge(V, start, mid, end);
}


```

###01/17 

#### Trie :palm_tree:
> when last char c was found by cur->M.count(c)\
> Do cur->M[c]->is_word = ture. \
> not cur->is_word = ture;\
> passing idx+1 in function call can be done as ++idx, not idx++
```cpp
 Trie() {}
    unordered_map<char, Trie*> M;
    bool is_word = false;;
    
    void insert_helper(string& word, Trie* cur, int idx){
      if (word.size() == 0) return;
      auto c = word[idx];
      if (!cur->M.count(c)) cur->M[c] = new Trie();
      if (idx == word.size()-1){
         cur->M[c]->is_word = true;
         return;
       }else{
        cur = cur->M[c];
        insert_helper(word, cur, ++idx);
       }
    }
      
    /** Inserts a word into the trie. */
    void insert(string word) {
      insert_helper(word, this, 0);
    }
    
    // return 0-> not found, 1->found, 2-> is_word
    int search_helper(string& word, Trie* root, int idx){
      if (word.size() == 0) return true;
      auto c = word[idx];
      if (!root->M.count(c)) return 0;
      if (idx == word.size()-1){
        if (root->M[c]->is_word) return 2;
        return 1;
      }else{
        root = root->M[c];
        return search_helper(word, root, idx+1);
      }
    }
    /** Returns if the word is in the trie. */
    bool search(string word) {
      return search_helper(word, this, 0) == 2;
    }
    
    /** Returns if there is any word in the trie that starts with the given prefix. */
    bool startsWith(string prefix) {
      return search_helper(prefix, this, 0) > 0;
    }

```
###01/14

### fenwick/segment tree
> index starting with 1 \
> add --> until (idx __+=__ idx & -idx ) >= range \
> sum --> until (idx __-=__ idx & -idx ) <= 0

```cpp
void add(vector<int>& sum, int idx, int val){
	while (idx < sum.size()){
		sum[idx]+=val; 
		idx = idx + ((idx) & -(idx));
	}
}

int getSum(vector<int>& sum, int idx){
   int ret = 0;
	while (idx > 0){
		cout << "idx: " << idx << endl;
		ret += sum[idx];
		idx-=(idx&(-idx));
	}
	return ret;
}

void add2D(vector<vector<int>>& sum, int row, int col, int val){   
   int new_row = row;
   while (new_row < sum.size()){
      int new_col = col;
      while (new_col < sum[0].size()){
         sum[new_row][new_col]+=val;
         new_col += (new_col & -(new_col));
      }
      new_row += (new_row & (-new_row));
   }
}

int getSum2D(vector<vector<int>>& sum, int row, int col){
   int ret = 0;
   int new_row = row;
   while(new_row >0){
      int new_col = col;
      while (new_col > 0){
         ret += sum[new_row][new_col];
         new_col -= (new_col & -(new_col));
      }
      new_row -=  (new_row & -(new_row));
   }
   return ret;
}


```

[max points problem](https://leetcode.com/problems/max-points-on-a-line/submissions/)
> memorize GCD. becareful the GCD may return 0 when both a and b are 0.\
> map(pair<...>, int) doesn't need hash functor. unordered_map<pair<..>, int> does.

```cpp
 int gcd(int a, int b){
      while (b) {
        int tmp = b;
        b = a%b;
        a = tmp;
      }
      return a;
    }
int maxPoints(vector<vector<int>>& points) {
       int ret = 0;  
       if (points.size() == 0) return ret;
       for (int i= 0; i<points.size(); i++){
         int local = 0, vertical = 0, overlap = 0, deltaX, deltaY;
         map<pair<int,int>, int> M;
         auto p = points[i];
         for (int j=i+1; j<points.size(); j++){
           auto q = points[j];
           if (p[0] == q[0] && p[1] == q[1]) overlap++;
           else{
             deltaX = q[0]-p[0];
             deltaY = q[1]-p[1];
             if (deltaX == 0) vertical++;
             else{
               auto common = gcd(deltaY, deltaX);
               if (common != 0) {
                 deltaX /= common;
                 deltaY /= common;
               }
               M[make_pair(deltaX,deltaY)]++;
             }
           }
         }
         for (auto m: M) local = max(local, m.second);
         local = max(local, vertical);
         ret = max(ret, local+overlap+1);
       }
       return ret;
    
}
```



# 01/11
__skyline_problem__
> Be extremely careful for **COMPARISON** 
> otherwise none-combined outputs may occur
```cpp
  vector<vector<int>> getSkyline(vector<vector<int>>& buildings) {
      vector<vector<int>> ret;
      buildings.push_back({-1,-1,-1});
      priority_queue<pair<int,int>> PQ;
      for (auto& B : buildings){
        while (PQ.size() && (B[0] == -1 || B[0] > PQ.top().second)){
          auto T = PQ.top(); PQ.pop();
          while (PQ.size() && T.second >= PQ.top().second) PQ.pop();
          if (PQ.size()){
            if (ret.size() && ret.back()[0] == T.second)
              ret.back()[1] = max(ret.back()[1], PQ.top().first);
            else
              ret.push_back({T.second, PQ.top().first});
          }else{
            ret.push_back({T.second, 0});
          }
        }
        if (B[1] == -1 ) break; 
        if (!PQ.size() || B[2] > PQ.top().first){ // avoid [0,3],[2,3] which should be [0,3]
          if (ret.size() && ret.back()[0] == B[0])
            ret.back()[1] = max(ret.back()[1], B[2]);
          else  
            ret.push_back({B[0], B[2]});
        }
        PQ.push({B[2],B[1]});
      }
      return ret;
```

# 01/09
> LC compile error if function doesn't return as expected. 
```cpp
 vector<int> twoSum(vector<int>& nums, int target) {
      unordered_map<int,int> M;
      for (int i=0; i< nums.size(); i++){
        if (M.count(nums[i])) return {i, M[nums[i]]};
        M[target-nums[i]] = i;
      }
      return {0,0};  // YL. Without this line. compile failed. 
    }
```

[missing range](https://leetcode.com/explore/interview/card/top-interview-questions-medium/103/array-and-strings/782/)
>   combine same logic into function\
>   when there is minus, considering overflow and use long long
```cpp

```


# 01/07  :ppalm_tree:
###### STOCK question. 
__DP approach for K transactions__\
__STATE approach for cool_down requirement__
> without cool dow, two status
> with cool down, three status

```cpp
int DP(vector<int>& P, int k){
	vector<int> score(P.size(), 0);
	int ret = 0;
   for (int kk=0; kk<k; kk++){
      int expense = P[0];
		for (int i=1; i<P.size(); i++){
			int pre_score = score[i];
			score[i] = max(score[i-1], P[i]-expense);
			expense = min(expense, P[i]-pre_score);
	   }
	}
	return score[P.size()-1];
}

int state(vector<int>& P){

   int hold = INT_MIN, empty = 0;
   int ret = 0;
   for (int i=0; i<P.size(); i++){
      int h = hold; int e = empty;
      empty = max(e, h+P[i]);
      hold = max(h, e-P[i]);
   }
   return max(hold,empty);
}

int state_cool_down(vector<int>& P){

   int idle = 0, empty = 0, hold = INT_MIN;
   for (int i=0; i<P.size(); i++){
      int old_idle = idle; int old_empty = empty; int old_hold = hold;
      idle = max(old_idle, old_hold+P[i]);
      empty = max(old_empty, old_idle);
      hold = max(old_hold, old_empty-P[i]);
   }
   return max(idle, empty);
}

}
```
# 01/06  :ppalm_tree:

###### 3 Sum review. 
>   to avoid duplication\
>   **outter loop:**  
>       __if (i && nums[i] == num[i-1]) continue;__\
>   **inner loop:**  
>       while (left < right && nums[left] == nums[left-1]) left++;
>       while (left < right && nums[right] == nums[right+1]) right--;

# 01/05  :palm_tree:
[Article: two mins break after question is given](https://leetcode.com/discuss/interview-experience/469785/Finally-I-Did-it.Thanks-Leetcode-Love-you-so-much-Microsoft-or-Offer) :racehorse:

[26. Remove Duplicates from Sorted Array easy](https://leetcode.com/problems/remove-duplicates-from-sorted-array/) :herb:
     
**Doesn't advance pointer again.**
```cpp
  int removeDuplicates(vector<int>& nums) {
        int len = nums.size(), i, j;
        for (i=-1, j=0 ;j<len; j++){
          if (i<0 || nums[i] != nums[j]){
            nums[++i] = nums[j]; // YL. don't advance j pointer as this is already done in for loop
          }  
        }
        return i+1;
    }
```
#### [ispired by stock IV solution using state machine](https://leetcode.com/problems/best-time-to-buy-and-sell-stock-with-cooldown/discuss/75928/Share-my-DP-solution-(By-State-Machine-Thinking))
[122. Best Time to Buy and Sell Stock. Unlimite transaction-- STATE MACHINE ](https://leetcode.com/problems/best-time-to-buy-and-sell-stock-ii/) :herb:
```cpp
 // easy solution:
   int stockNoRestriction(vector<int>& prices){
      int profit = 0;
      for (int i=1; i< prices.size(); i++){
           auto diff = prices[i] - prices[i-1];
           profit += diff > 0 ? diff : 0;
      }
      return profit;
    }
    
 // state machine
 int maxProfit(vector<int>& prices) {
      int len  = prices.size();
      if (!len) return 0;
      vector<int> NotOwned(len, 0);
      vector<int> Owned(len, 0);
      NotOwned[0] = 0;  Owned[0] = -prices[0];
      for (int i=1; i<len; i++){
        NotOwned[i] = max(NotOwned[i-1], Owned[i-1] + prices[i]);
        Owned[i] = max(Owned[i-1], NotOwned[i-1] - prices[i]);
      }
      return max(NotOwned[len-1], Owned[len-1]);
    }
```
[309. Best Time to buy and sell stock. unlimite with COOL DOWN](https://leetcode.com/problems/best-time-to-buy-and-sell-stock-with-cooldown/):herb:
```cpp
  int buy = 0;
      int sell = INT_MIN;
      int cool = 0;
      for (int i=0; i<prices.size(); i++){
        int new_buy = max(buy, cool);
        int new_sell = max(sell, buy-prices[i]);
        int new_cool = sell + prices[i];
        buy = new_buy, sell = new_sell, cool = new_cool;
      }
      return max(buy,cool);
```


#### Best time to buy and sell stock using DP.
[123. Best Time to Buy and Sell Stock: At most K transactions](https://leetcode.com/problems/best-time-to-buy-and-sell-stock-iii/) :herb:
```cpp
  int maxProfit(int k, vector<int>& prices) {
        int len = prices.size();
        if (len < 2) return 0;
        if (k >= len/2) return stockNoRestriction(prices);
        int maxBuy = k;
        vector<int> cur_scores(len, 0);
        //vector<int> pre_scores(len, 0);
      
        for (int i=0; i<maxBuy; i++){
          int tmpMax = -prices[0];
          for (int j = 1; j<len; j++){
            int prev = cur_scores[j];
            cur_scores[j] = max(cur_scores[j-1], prices[j]+tmpMax); // check if curent sell get best profit so far
            //tmpMax = max(tmpMax, pre_scores[j] - prices[j]);  // check if current buy make best potiential for future
            tmpMax = max(tmpMax, prev - prices[j]);  // check if current buy make best potiential for future
          }
          //cout << "transaction : " << i+1 << " " <<  cur_scores[len-1] << endl;
          //swap(pre_scores, cur_scores);
        }
        return cur_scores[len-1];
    }
     
    int stockNoRestriction(vector<int>& prices){
      int profit = 0;
      for (int i=1; i< prices.size(); i++){
           auto diff = prices[i] - prices[i-1];
           profit += diff > 0 ? diff : 0;
      }
      return profit;
    }
``` 

[Rotate Array](https://leetcode.com/explore/featured/card/top-interview-questions-easy/92/array/646/) :herb:
###### Be careful about circling. :moon:
```cpp
 void rotate(vector<int>& nums, int k) {
      int len = nums.size();
      if ((k = k % len) < 1) return; 
      
      int visit = 0;
      for (int i=0; i<nums.size() && visit < len; i++){ // stop when visit == len 
        int idx = i;
        int val = nums[idx];
        while (true) {
          int new_idx = (idx+k)%len;
          int new_val = nums[new_idx];
          nums[new_idx]=val;
          //cout << new_idx << " ---> " << val << endl;
          visit++;
          idx = new_idx;
          val = new_val;
          if (idx == i) break; // YL: break from circling 
        }
      }
    }

```

[3sum without duplication](https://leetcode.com/explore/interview/card/top-interview-questions-medium/103/array-and-strings/776/) :herb:
###### Be careful about duplication, see comments :moon:
```cpp
 vector<vector<int>> threeSum(vector<int>& nums) {
      
        vector<vector<int>> ret;
        int len = nums.size();
        if (len < 3) return ret;
        sort(nums.begin(), nums.end());
      
        for (int i=0; i<len-2; i++){
          //if (i+3 <len-2 && nums[i] == nums[i+3]) continue; //YL: doesn't
          if (i && nums[i] == nums[i-1]) continue; //YL This works. 
          int remain = -nums[i];
          cout << "i: " << i << endl;
          int left = i+1, right = len-1;
          while (left < right){
            int sum = nums[left]+nums[right];
            if (sum < remain){
              left++;
              while (left < right && nums[left] == nums[left-1]) left++;
            }else if (sum > remain){
              right--;
              while (right > left && nums[right] == nums[right+1]) right--;
            }else{
              ret.push_back({nums[i], nums[left], nums[right]});
              left++; right--;
              while (left < right && nums[left] == nums[left-1]) left++;
              while (right > left && nums[right] == nums[right+1]) right--;
            }
          }
        }
        return ret;
    }
```

[Set matrix zero in place](https://leetcode.com/explore/interview/card/top-interview-questions-medium/103/array-and-strings/777/) :herb:
```cpp
void setZeroes(vector<vector<int>>& matrix) {
      
      // if first row has zero
      bool firstRowHasZero =   false;
      for (int j=0; j<matrix[0].size(); j++){
        if (matrix[0][j] == 0)
           firstRowHasZero = true;
      }  
      
      // first row and first col
      for (int i=1; i<matrix.size(); i++){
        for (int j= 0; j<matrix[0].size(); j++){
          if (matrix[i][j] == 0){
            matrix[0][j] = 0;
            matrix[i][0] = 0;
          }
        }
      }
      
      // reset each row
      for (int i=1; i<matrix.size(); i++){
        if (matrix[i][0] == 0)
          for (int j= 0; j<matrix[0].size(); j++)
            matrix[i][j] = 0;
      }  
      
      // reset each col 
      for (int j=0; j<matrix[0].size(); j++){
        if (matrix[0][j] == 0){
          for (int i=1; i<matrix.size(); i++){
            matrix[i][j] = 0;
          }
        }
      }  
      
      //reset first row
      if (firstRowHasZero)      
        for (int j=0; j<matrix[0].size(); j++)
          matrix[0][j] = 0;
    }
```

[group Anagrams](https://leetcode.com/explore/interview/card/top-interview-questions-medium/103/array-and-strings/778/) :herb:
```cpp
/*
Given an array of strings, group anagrams together.
Input: ["eat", "tea", "tan", "ate", "nat", "bat"],
Output:
[
  ["ate","eat","tea"],
  ["nat","tan"],
  ["bat"]
]
*/
vector<vector<string>> groupAnagrams(vector<string>& strs) {
      unordered_map<string, vector<string>> m;
      for (auto w: strs){
        auto temp = w;
        sort(temp.begin(), temp.end());
        m[temp].push_back(w);
      }
      vector<vector<string>> ret;
      for (auto mm: m){
        ret.push_back(mm.second);
      }
      return ret;
    }
```
    
[Product of Array Except Self](https://leetcode.com/explore/interview/card/top-interview-questions-hard/116/array-and-strings/827/) :herb:
```cpp
  /*
    Given an array nums of n integers where n > 1,  return an array output such that output[i] is equal to the product of all the elements of nums except nums[i].
    Example:

Input:  [1,2,3,4]
Output: [24,12,8,6]
Note: Please solve it without division and in O(n).

Follow up:
Could you solve it with constant space complexity? (The output array does not count as extra space for the purpose of space complexity analysis.)
*/
  vector<int> productExceptSelf(vector<int>& nums) {
      vector<int> ret(nums.size(), 0);
      ret[0] = 1;
      for (int i=1; i<nums.size(); i++){
        ret[i] = nums[i-1]*ret[i-1];
      }
      int temp = 1;
      for (int i=nums.size()-2; i>=0; i--){
         temp*=nums[i+1];
         ret[i]*=temp;
      }
      return ret;
    }
```
[Spriral Matrix](https://leetcode.com/explore/interview/card/top-interview-questions-hard/116/array-and-strings/828/) :herb:
###### Need redo.  di = (di+1)%4 :cow:  time O(n). space O(n)
```cpp
/*
Spiral Matrix
Solution
Given a matrix of m x n elements (m rows, n columns), return all elements of the matrix in spiral order.

Example 1:

Input:
[
 [ 1, 2, 3 ],
 [ 4, 5, 6 ],
 [ 7, 8, 9 ]
]
Output: [1,2,3,6,9,8,7,4,5]
Example 2:

Input:
[
  [1, 2, 3, 4],
  [5, 6, 7, 8],
  [9,10,11,12]
]
Output: [1,2,3,4,8,12,11,10,9,5,6,7]
*/

//LC 解法1. Simulation 

 public List<Integer> spiralOrder(int[][] matrix) {
        List ans = new ArrayList();
        if (matrix.length == 0) return ans;
        int R = matrix.length, C = matrix[0].length;
        boolean[][] seen = new boolean[R][C];
        int[] dr = {0, 1, 0, -1};
        int[] dc = {1, 0, -1, 0};
        int r = 0, c = 0, di = 0;
        for (int i = 0; i < R * C; i++) {
            ans.add(matrix[r][c]);
            seen[r][c] = true;
            int cr = r + dr[di];
            int cc = c + dc[di];
            if (0 <= cr && cr < R && 0 <= cc && cc < C && !seen[cr][cc]){
                r = cr;
                c = cc;
            } else {
                di = (di + 1) % 4;
                r += dr[di];
                c += dc[di];
            }
        }
        return ans;
}

// LC 解法2. 层层递进
  public List < Integer > spiralOrder(int[][] matrix) {
        List ans = new ArrayList();
        if (matrix.length == 0)
            return ans;
        int r1 = 0, r2 = matrix.length - 1;
        int c1 = 0, c2 = matrix[0].length - 1;
        while (r1 <= r2 && c1 <= c2) {
            for (int c = c1; c <= c2; c++) ans.add(matrix[r1][c]);
            for (int r = r1 + 1; r <= r2; r++) ans.add(matrix[r][c2]);
            if (r1 < r2 && c1 < c2) {
                for (int c = c2 - 1; c > c1; c--) ans.add(matrix[r2][c]);
                for (int r = r2; r > r1; r--) ans.add(matrix[r][c1]);
            }
            r1++;
            r2--;
            c1++;
            c2--;
        }
        return ans;
    }
```

[4sumII](https://leetcode.com/explore/interview/card/top-interview-questions-hard/116/array-and-strings/829/):herb:

```cpp
/*
Given four lists A, B, C, D of integer values, compute how many tuples (i, j, k, l) there are such that A[i] + B[j] + C[k] + D[l] is zero.

To make problem a bit easier, all A, B, C, D have same length of N where 0 ≤ N ≤ 500. All integers are in the range of -228 to 228 - 1 and the result is guaranteed to be at most 231 - 1.

Example:

Input:
A = [ 1, 2]
B = [-2,-1]
C = [-1, 2]
D = [ 0, 2]

Output:
2

Explanation:
The two tuples are:
1. (0, 0, 0, 1) -> A[0] + B[0] + C[0] + D[1] = 1 + (-2) + (-1) + 2 = 0
2. (1, 1, 0, 0) -> A[1] + B[1] + C[0] + D[0] = 2 + (-1) + (-1) + 0 = 0
*/
int fourSumCount(vector<int>& A, vector<int>& B, vector<int>& C, vector<int>& D) {
      unordered_map<int, int> M;
       for (auto a : A) {
         for (auto b: B){
            M[a+b]++;
         }
       }
       int ret = 0;
       for (auto c : C) {
         for (auto d: D){
            if (M.count(-c-d))
              ret += M[-c-d];
         }
       }
       return ret;
    }
```




[380. Insert Delete GetRandom O(1)] (https://leetcode.com/problems/insert-delete-getrandom-o1)

```cpp
 /** Inserts a value to the set. Returns true if the set did not already contain the specified element. */
    bool insert(int val) {
      if (M.count(val)) return false;
      V.push_back(val);
      M[val] = V.size()-1;
      return true;
    }
    
    /** Removes a value from the set. Returns true if the set contained the specified element. */
    bool remove(int val) {
      if (!M.count(val)) return false;
      int last_val  = V[V.size()-1];
      
      // switch val with last element of V
      int idx = M[val];
      V[idx] = last_val;
      
      // delete last element
      M[last_val] = idx;
      V.pop_back();  // <======    V size is reduced by one
      
      M.erase(val);
      return true;
    }
    
    /** Get a random element from the set. */
    int getRandom() {
      int idx = rand()%(V.size());
      return V[idx];
    }
  
    vector<int> V;
    unordered_map<int, int> M;  // <====== YL unordered_map<int, vector<int>::iterator> M; doesn't work.
```

:camel:
[172. Factorial Trailing Zeroes] (https://leetcode.com/problems/factorial-trailing-zeroes/)
**Read problem and think carefully** only for loop ( 0 ... lg(n) )  does the job.

```cpp
int trailingZeroes(int n) {
   int result = 0;
   /* only a sinlge 
   for(long long i=5; n/i>0; i*=5){
     result += (n/i);
   }
return result;
   }

```


[ Excel Sheet Column Number]

```cpp
int titleToNumber(string s) {
   // use long to make LC testcase happy. notes that ret is still int 
   long digit = 1, ret = 0;
   for (int i=s.size()-1; i>=0; i--){
      ret+= (s[i]-'A'+1) * digit;
      digit = digit*26;
   }
   return ret;
}
```
[Pow(x,n)](https://leetcode.com/problems/powx-n/)
### How to further speedup
__[better way;](https://leetcode.com/problems/powx-n/discuss/19544/5-different-choices-when-talk-with-interviewers)__
```cpp
 
double pow(double x, int n) {
   if (x == 0.0) return x;
   if (n < 0){
      if (n==INT_MIN)
         return myPow(x,n+1)/x; // YL: coverting -max to max trap.
      else  
         return 1.0/myPow(x,-n);
      }

   double ret = 1.0;
   double step = 1;
   double step_val = x;
   while (n > 0){
      //cout << "step_val: " << step_val << endl;
      ret = ret*step_val;
      n -= step;
      step_val = step_val*step_val;
      step *= 2;
      if (step>n){  //YLYL how to further speedup
         step = 1 ;
         step_val = x ;
      }
   }    
   return ret;
   }
```


[142 Linked List Cycle II]("https://leetcode.com/problems/linked-list-cycle-ii/")

[287 Find the Duplicate Number]("https://leetcode.com/problems/find-the-duplicate-number/")

[Number of Islands](https://leetcode.com/explore/interview/card/top-interview-questions-medium/108/trees-and-graphs/792/)
DFS/BFS/UNION_FIND see lc solution.

[Longest Increasing subsquence](https://leetcode.com/explore/interview/card/top-interview-questions-medium/111/dynamic-programming/810/)
1. DP N*N
2. lower_bound: Nlog(N)
3. 



[251. Flatten 2D Vector](https://leetcode.com/problems/flatten-2d-vector/)
*** Be very careful about null dereference


```cpp
class Vector2D {
public:
   
    vector<vector<int>>::iterator outer_it, e ;
    vector<int>::iterator cur_it;
     
    Vector2D(vector<vector<int>>& v) {
      outer_it = v.begin();
      e = v.end();
      if (outer_it !=v.end()) cur_it = outer_it->begin();// <======= null check
    }
    
    int next() {
       hasNext();
       return *(cur_it++);
    }
    
    bool hasNext() {
       while (outer_it != e && cur_it == outer_it->end()) {
         outer_it++;
         cout << "advancing iterator\n";
         if (outer_it != e) cur_it = outer_it->begin(); // <====== null check
       }
       return outer_it != e;
    }
    
};
```

### 380. Insert Delete GetRandom
Need redo as O(1) is needed.### Missing Ranges

1. trick: overflow. using long for diff
2. lower ---> nums[0]
3  loop:  nums[0] ---> nums[last]
4  nums[last] ---> upper

```cpp
/*
    static string getRange(int left, int right){
      if (left == right) return to_string(left);
      return to_string(left) + "->" + to_string(right);
      
    }
    vector<string> findMissingRanges(vector<int>& nums, int lower, int upper) {
      vector<string> ret;
      
      // case empty array
      if (nums.size() == 0)  {ret.push_back(getRange(lower, upper)); return ret;}
      
      // case: before lower
      if (lower<nums[0]) ret.push_back(getRange(lower, nums[0]-1));
      
      int left = nums[0];
      for (int i=1; i<nums.size(); i++){
        if (i>0 && nums[i] == nums[i-1]) continue;
        long diff = (long)nums[i] - (long)left;  //============> using long
        if (diff == 1){
          left++;
        }else if (diff == 2){
          ret.push_back(to_string(left+1));
          left = nums[i];
        }else{
          ret.push_back(to_string(left+1)+ "->" + to_string(nums[i]-1));
          left = nums[i];
        }
      }
      // case: after upper 
      if (nums[nums.size()-1] < upper){
        ret.push_back(getRange(nums[nums.size()-1]+1, upper));
      }
      return ret;
    }


```### To do: quickSelect

:sparkles: :sparkles:
[912 sort_array](https://leetcode.com/problems/sort-an-array/)

### quick sort notes:
1. use right node as pivot
2. for loop --> cur_index 
3. avdanced left when A[cur_idx] < pivot 
4. swap(A[left], A[right]) return left

### mergeSort notes
1. topDown and bottomUp use same combine function.
2. 


```cpp
    //============================
    // Quick sort
    //============================
    void shuffle(vector<int>& nums) {
       for (int i=0; i <nums.size(); i++){
         auto idx = rand()%(i+1);
         swap(nums[idx], nums[i]);
       }
    }
    
    int partition(vector<int>& nums, int left, int right){
       int val = nums[right];
       for (int i=left; i<right; i++){
         if (nums[i] <= val){
           swap(nums[left], nums[i]);
           left++;
         }
       }
       swap(nums[left], nums[right]);
       return left;
    }
  
    void quickSort(vector<int>& nums, int lo, int hi){
       if (lo < hi) {
          auto p = partition(nums, lo, hi);
          quickSort(nums, lo, p-1);
          quickSort(nums, p+1, hi);
       }
    }
  
   //============================
    // MergeSort combine fucntion. 
    // for both topDown and bottom
    //============================
   
    void combine(vector<int>& nums, int start, int end, int mid) {
       if (start > mid || mid > end) return;
       int left = start, right = mid+1;
       vector<int> tmp; 
       while (left <= mid || right <= end){
         if (left <=mid && right <= end){
           if (nums[left]<=nums[right])
             tmp.push_back(nums[left++]);
            else
             tmp.push_back(nums[right++]);
         }else if (left <= mid){
             tmp.push_back(nums[left++]);
         }else{
             tmp.push_back(nums[right++]);
         }
       }
       for (int i=start, a = 0; i<=end; i++) nums[i] = tmp[a++];
    }
  
   //============================
    // MergeSort topDown
    //============================
    void mergeSortTopDown(vector<int>& nums, int start, int end){
       int mid = start+(end-start)/2;
       mergeSortTopDown(nums, start, mid);
       mergeSortTopDown(nums, mid+1, end);
       combine(nums, start, end, mid);
    }
  
    //============================
    // MergeSort bottomUp
    //============================
  
     void mergeSortBottomUp(vector<int>& nums, int start, int end){
       for (int step = 1; step < nums.size(); step*=2){
         for (int j=0; j<=end; j+=step*2){
           combine(nums, j, min(end, j+step*2-1),j+step-1);
         }
       }
     }
  
    vector<int> sortArray(vector<int>& nums) {
      //shuffle(nums);
      //quickSort(nums, 0, nums.size()-1);
      //mergeSortTopDown(nums, 0, nums.size()-1);
      mergeSortBottomUp(nums, 0, nums.size()-1);
      return nums;
    }
      
```


## Three color sort. 

```cpp
void sortColors(vector<int>& nums) {
        int cur=0, left=-1, right = nums.size();
        while (cur < right){
          if (nums[cur] == 0){
            swap(nums[cur++],nums[++left]);  // move both left and cur
          }else if (nums[cur] == 2){
            swap(nums[cur],nums[--right]); ;// ===> only move right
          }else{
            cur++; //===> only move cur
          }
        }
    }
    ```### phone numer to message. 

```cpp 

    // backtracking
    void helper(vector<string>& ret, vector<vector<char>>& S, string& cur, string& digits, int len){
      
       if (len == digits.size()){
         ret.push_back(cur);
         return;
       }
       auto& C = S[digits[len]-'2'];
       for (auto c: C){
         cur.push_back(c);
         helper(ret, S, cur, digits, len+1);
         cur.pop_back();
       }
    }
    vector<string> letterCombinations(string digits) {
      if (digits.size() == 0) return {};
      vector<vector<char>> S = {{'a','b','c'}, {'d','e','f'},{'g','h','i'},{'j','k','l'},{'m','n','o'},{'p','q','r','s'},{'t','u','v'},{'w','x','y','z'}};
      string cur = "";
      vector<string> ret;
      helper(ret, S, cur, digits, 0);
      return ret;
    }

    
    // iteration
    vector<string> letterCombinations(string digits) {
      if (digits.size() == 0) return {};
      vector<vector<char>> S = {{'a','b','c'}, {'d','e','f'},{'g','h','i'},{'j','k','l'},{'m','n','o'},{'p','q','r','s'},{'t','u','v'},{'w','x','y','z'}};
      vector<string> ret = {""};
      for (auto d : digits){
        int cur_size = ret.size();  // :camel: use current container size in  loop  :camel:

        for (auto c: S[d-'2']){
          for (int i=0; i< cur_size; i++){
             ret.push_back(ret[i]+c);
          }
        }
        ret.erase(ret.begin(), ret.begin()+cur_size);
      }
      return ret;
    }
```## The failures and reverses which await men - and one after another sadden the brow of youth- add a dignity to the prospect of human life, which no Arcadian success would do.
:camel: :camel: :camel:  



### Binary Tree Zigzag Level Order Traversal 
```cpp
/*

Given binary tree [3,9,20,null,null,15,7],
 3
/ \
9  20
/  \
15   7

return its zigzag level order traversal as:

[
[3],
[20,9],
[15,7]
]

vector<TreeNode*> extract(vector<TreeNode*> nodes, int flip, vector<vector<int>>& V){
   vector<int> v;
   vector<TreeNode*> new_nodes;
   for (auto& n: nodes){
      if (n->left) new_nodes.push_back(n->left);
      if (n->right) new_nodes.push_back(n->right);
      if (flip == 1) v.push_back(n->val);
      else v.insert(v.begin(),n->val);
   }
   V.push_back(v);
   return new_nodes;
}

vector<vector<int>> zigzagLevelOrder(TreeNode* root) {
   if (!root) return {};
   int ori = 1;
   vector<vector<int>> ret;
   vector<TreeNode*> nodes;
   nodes.push_back(root);
   while (nodes.size()){
      nodes = extract(nodes, ori,ret); 
      ori*=-1;
   }  
   return ret;
}
```

### Inorder Successor in BST
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


### Number of islands
* dfs
* count++ whenever '1' is found
* update input matrix (should ask if allowed)


### populate next right pointers in EAch node :camel:
```cpp
Node* connect(Node* root) {
   Node* left_most = root;
   while (left_most){
      auto head = left_most;
      while (head){
         if (head->left){
            head->left->next = head->right;
            if (head->next) head->right->next = head->next->left;
         }
         head = head->next;
      }
      left_most = left_most->left;
   }
   return root;
}
```

# morris travesal :camel: :camel:

```cpp
vector<int> inorderTraversal(TreeNode* root) {
   vector<int> ret;
   while (root){
      if (root->left){
         TreeNode* last = root->left;
         while (last->right && last->right != root){
            last = last->right;
         }
         if (last->right == NULL){
            last->right = root;
            root = root->left;
         }else{
            last->right = NULL;
            ret.push_back(root->val);
            root = root->right;
         }
      }else{
         ret.push_back(root->val);
         root = root->right;
      }
   }
   return ret;
}

```




<span style="color:green">The people who get on in this world are the people who get up and look for circumstances they want, and if they cannot find them, make them.</span>  

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
