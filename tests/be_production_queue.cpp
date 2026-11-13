// Compile with -DBE_QUEUE_HEADER='"path/to/the/actual/aged_object_queue.hpp"'.
#include BE_QUEUE_HEADER
#include <cassert>
int main(){
  AgedObjectQueue<int> q(5);q.push(71);
  for(int attempt=0;attempt<5;++attempt){assert(!q.empty());assert(q.pop_increment_age()==71);}
  assert(q.empty());
  // Refusing a new observation does not cancel four remaining uses of an old one.
  q.push(11);assert(q.pop_increment_age()==11);
  for(int remaining=0;remaining<4;++remaining){assert(!q.empty());assert(q.pop_increment_age()==11);}
  assert(q.empty());
  // Failure of an update cannot be reported to this queue: popping already ages it.
  q.push(3);q.pop_increment_age();q.pop_increment_age();q.pop_increment_age();q.pop_increment_age();q.pop_increment_age();assert(q.empty());
}
