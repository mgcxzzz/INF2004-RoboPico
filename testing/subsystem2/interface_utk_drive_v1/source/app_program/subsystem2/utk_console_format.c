/* Bounded streaming formatting for the controller's USB CSV logs. No
 * stdio FILE state, malloc, network calls or scheduler waits are involved. */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
extern void utk_console_write(const char *data,unsigned count);
typedef struct {char bytes[128];unsigned used;int count;} Writer;
static void flush(Writer *w) {
    if(w->used){utk_console_write(w->bytes,w->used);w->used=0;}
}
static void emit(Writer *w,char c) {
    w->bytes[w->used++]=c;++w->count;if(w->used==sizeof(w->bytes))flush(w);
}
static unsigned number(char *out,uint64_t value,unsigned base,bool upper) {
    char reverse[65];unsigned n=0;
    do {unsigned d=(unsigned)(value%base);value/=base;reverse[n++]=(char)(d<10?'0'+d:(upper?'A':'a')+d-10);}while(value);
    for(unsigned i=0;i<n;++i){out[i]=reverse[n-i-1];}
    return n;
}
static unsigned decimal(char *out,double value,unsigned precision) {
    if(value!=value){out[0]='n';out[1]='a';out[2]='n';return 3;}
    if(value>9.0e15){out[0]='o';out[1]='v';out[2]='f';return 3;}
    uint64_t scale=1;for(unsigned i=0;i<precision;++i)scale*=10;
    if(value>9.0e15/(double)scale){out[0]='o';out[1]='v';out[2]='f';return 3;}
    uint64_t rounded=(uint64_t)(value*(double)scale+0.5);
    unsigned n=number(out,rounded/scale,10,false);
    if(precision){out[n++]='.';uint64_t part=rounded%scale;uint64_t divisor=scale/10;
        for(unsigned i=0;i<precision;++i){out[n++]=(char)('0'+part/divisor);part%=divisor;divisor/=10;}}
    return n;
}
int utk_printf(const char *format,...)
{
    Writer w={0};va_list args;va_start(args,format);
    for(const char *p=format;*p;++p) {
        if(*p!='%'){emit(&w,*p);continue;}
        ++p;if(*p=='%'){emit(&w,'%');continue;}
        bool zero=false,left=false,plus=false;
        while(*p=='0'||*p=='-'||*p=='+'){if(*p=='0')zero=true;else if(*p=='-')left=true;else plus=true;++p;}
        unsigned width=0,precision=6;
        while(*p>='0'&&*p<='9'){width=width*10+(unsigned)(*p-'0');++p;if(width>64)width=64;}
        if(*p=='.'){precision=0;++p;while(*p>='0'&&*p<='9'){precision=precision*10+(unsigned)(*p-'0');++p;if(precision>6)precision=6;}}
        unsigned length=0;
        while(*p=='l'){++length;++p;}
        if(*p=='z'){length=sizeof(size_t)==8?2:1;++p;}
        char buffer[96];const char *text=buffer;unsigned size=0;char sign=0;
        if(*p=='s'){text=va_arg(args,const char *);if(!text)text="(null)";while(text[size])++size;}
        else if(*p=='c'){buffer[0]=(char)va_arg(args,int);size=1;}
        else if(*p=='f'){double value=va_arg(args,double);if(value<0){sign='-';value=-value;}else if(plus)sign='+';size=decimal(buffer,value,precision);}
        else if(*p=='d'||*p=='i'||*p=='u'||*p=='x'||*p=='X'||*p=='p') {
            uint64_t value;
            if(*p=='d'||*p=='i') {int64_t signed_value=length>1?va_arg(args,long long):length?va_arg(args,long):va_arg(args,int);
                value=(uint64_t)signed_value;if(signed_value<0){sign='-';value=0-value;}else if(plus)sign='+';}
            else if(*p=='p')value=(uintptr_t)va_arg(args,void *);
            else value=length>1?va_arg(args,unsigned long long):length?va_arg(args,unsigned long):va_arg(args,unsigned);
            size=number(buffer,value,(*p=='x'||*p=='X'||*p=='p')?16:10,*p=='X');
        } else {emit(&w,'%');if(*p)emit(&w,*p);else break;continue;}
        unsigned occupied=size+(sign?1U:0U),padding=width>occupied?width-occupied:0;
        if(!left&&!zero)while(padding){emit(&w,' ');--padding;}
        if(sign)emit(&w,sign);
        if(!left&&zero)while(padding){emit(&w,'0');--padding;}
        for(unsigned i=0;i<size;++i)emit(&w,text[i]);
        if(left)while(padding){emit(&w,' ');--padding;}
    }
    va_end(args);flush(&w);return w.count;
}
int utk_putchar(int c){char value=(char)c;utk_console_write(&value,1);return (unsigned char)value;}
int utk_puts(const char *text){unsigned n=0;while(text[n])++n;utk_console_write(text,n);utk_putchar('\n');return (int)n+1;}
