#include <string.h>

#define LINE_LEN 64
char inbuf[LINE_LEN]; // used to copy the serial buffer out so we don't lose the data when more comes in

typedef int16_t item;
typedef void (*nativeFun)(item*);

#define MAX_STACK 32
item dataStack[MAX_STACK];
int8_t dataStackIx = -1; //the index of the most recently-pushed item on dataStack

#define W_NATIVE 1
#define W_USER 2
#define W_VARIABLE 4
#define W_IMMEDIATE 128

typedef struct sDictEntry {
  int8_t type;
  int8_t nameLen;
  char name[8];
  nativeFun fn;
  item data;
} dictEntry;

//Fortunately, sizeof(int) == sizeof(nativeFun), so we can put
//function pointers on the stack without them getting truncated
void stackPush(item t){
  if (dataStackIx == MAX_STACK-1) {
    dataStackIx = -1;
    PTL("Stack overflow; stack cleared");
  } else {
    dataStack[++dataStackIx] = t;
  }
}

void stackPop() {
  if (dataStackIx < 0) {
    PTL("Popped an empty stack");
  } else {
    dataStackIx--;
  }
}

item* stackTop() {
  return dataStackIx < 0 ? NULL : dataStack+dataStackIx;
}

int16_t itemAsInt(){
    return (int16_t)(*stackTop());
}

dictEntry* itemAsDictEntry(){
    return (dictEntry*)(*stackTop());
}

void showStack(item*){
    for(int i = 0; i <= dataStackIx; i++) {
        PT((int16_t)dataStack[i]);
        PT(" ");
    }
    PTL();
}

void pushItem(item* it){
  stackPush(*it);
}

void stackPlus(item*){
    int16_t result = itemAsInt();
    stackPop();
    result += itemAsInt();
    stackPop();
    stackPush((item)result);
}

void stackDrop(item*){
    item* item = stackTop();
    if (item){
        PTL((int16_t)(*item));
    }
    stackPop();
}

#define MAX_DICT 16
dictEntry dictionary[MAX_DICT] = {
    {W_NATIVE, 2, ".s     ", showStack, -1},
    {W_NATIVE, 1, "+      ", stackPlus, -1},
    {W_NATIVE, 1, ".      ", stackDrop, -1},
};

int8_t dictLen = 3;

void addNativeFun(const char *name, nativeFun fn){
    int nameLen = strlen(name);
    dictionary[dictLen].type = W_NATIVE;
    dictionary[dictLen].nameLen = nameLen;
    memcpy(dictionary[dictLen].name, name, nameLen);
    dictionary[dictLen].fn = fn;
    dictionary[dictLen].data = -1;
    dictLen++;
}

void lookupWord(int start, int symLen) {
  // if the word doesn't exist, put -1 on the stack
  // if it does, put the function pointer on the stack
  for(int i = dictLen-1; i >= 0; i--){
    dictEntry* entry = &(dictionary[i]);
    if (symLen == entry->nameLen &&
        0 == memcmp(entry->name, inbuf+start, symLen)) {
      stackPush((item)entry);
      return;
    }
  }
  stackPush((item)(-1));
}

int itemIsTruthy(){
    return itemAsInt() != -1;
}

void executeItem(){
    dictEntry* topItem = itemAsDictEntry();
    stackPop();
    topItem->fn(&(topItem->data));
}

item stackPopToNative(){
    item result = itemAsInt();
    stackPop();
    return result;
}

int slurpNumber(int start, int symLen){
    int16_t runningTotal = 0;
    int16_t sign = 1;
    for(int i = start; i < start+symLen; i++){
        char c = inbuf[i];
        if (c=='-'){
            sign=-1;
        } else if (isDigit(c)) {
            runningTotal *= 10;
            runningTotal += c-'0';
        } else {
            return 0;
        }
    }
    stackPush((item)(sign*runningTotal));
    return 1;
}

char* charCursor;
typedef struct s_slice {
  nativeFun fn;
  item data;
} slice;
slice execQueue[32]; // arbitrary symbol limit. The most we can get per line is 32.
slice* execCursor;

void interpret(char *serial, int len){
  memset(inbuf, '\0', LINE_LEN);
  memcpy(inbuf, serial, len);
  int start = 0;
  slice* execQueueEnd = execQueue;

  // Skip any leading whitespace
  while (start < len) {
    while (!isGraph(inbuf[start]) && inbuf[start]) { start++; }
    int symLen = 0;
    // Find how long this symbol is
    while (isGraph(inbuf[start+symLen]) && inbuf[start+symLen]){ symLen++; }
    if (symLen > 0) {
        lookupWord(start, symLen); //leaves the dict entry on the stack
        if (itemIsTruthy()) {
          dictEntry* entry = itemAsDictEntry();
          stackPop();
          if (W_IMMEDIATE & entry->type){
            entry->fn(entry->data);
          } else {
            //add a slice to the execQueue so that we can execute it shortly
            execQueueEnd->fn = entry->fn;
            execQueueEnd->data = entry->data;
            execQueueEnd++;
          }
        } else {
            stackPop(); //removes the -1 dict entry from the stack
            if(slurpNumber(start, symLen)) {
              // we've got a number on the stack, record the function to push it
              execQueueEnd->fn = pushItem;
              //pushItem will be called with the *address* of the data field, not the contents.
              //This lets us pass an array when we figure out how to handle an array length
              execQueueEnd->data = itemAsInt();
              execQueueEnd++;
              stackPop();
            } else {
                PT("e Unknown word: ");
                for(int i = start;i < start+symLen; i++){ PT(inbuf[i]); }
                PT(" (");
                for(int i = start; i < start+symLen; i++){ PT((int)inbuf[i]); PT(" "); }
                PTL(")");
                break;
            }
        }
    }
    start += symLen;
  }
  // Now the execQueue is full, we can iterate over it and actually execute our
  // bytecode
  execCursor = execQueue;
  while(execCursor != execQueueEnd){
    execCursor->fn(&(execCursor->data));
    execCursor++;
  }
}