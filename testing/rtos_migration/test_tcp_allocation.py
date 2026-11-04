"""Compile the PINNED upstream allocator under old/new options; no network I/O."""
from pathlib import Path
import os,shlex,subprocess
ROOT=Path(__file__).resolve().parents[2]
sdk=Path(os.environ.get('PICO_SDK_PATH',ROOT.parent/'.buddy2-tools/pico-sdk'))
text=(sdk/'lib/lwip/src/core/tcp_out.c').read_text(encoding='utf-8')
start=text.index('#if TCP_OVERSIZE\nstatic struct pbuf *\ntcp_pbuf_prealloc')
end=text.index('#endif /* TCP_OVERSIZE */',start)+len('#endif /* TCP_OVERSIZE */')
allocator=text[start:end]
out=ROOT/'testing/rtos_migration/build-host';out.mkdir(exist_ok=True)
source=out/'upstream_tcp_allocator.c'
source.write_text('''#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef uint16_t u16_t;typedef uint8_t u8_t;typedef int pbuf_layer;
struct pbuf{u16_t len,tot_len;void *next;};struct tcp_pcb{unsigned flags;void *unsent,*unacked;};
static unsigned allocated;
#define LWIP_ASSERT(m,c) assert(c)
#define LWIP_UNUSED_ARG(v) ((void)(v))
#define LWIP_MIN(a,b) ((a)<(b)?(a):(b))
#define LWIP_MEM_ALIGN_SIZE(v) (((v)+3)&~3)
#define TCP_OVERSIZE_CALC_LENGTH(v) ((v)+TCP_OVERSIZE)
#define TCP_WRITE_FLAG_MORE 2
#define TF_NODELAY 1
#define PBUF_RAM 0
static struct pbuf packet;
static struct pbuf *pbuf_alloc(pbuf_layer layer,u16_t length,int type){
 (void)layer;(void)type;allocated+=length;packet=(struct pbuf){length,length,0};return &packet;}
'''+allocator+'''
int main(void){struct tcp_pcb pcb={.flags=TF_NODELAY};u16_t oversize=0;(void)oversize;(void)pcb;
 for(unsigned n=0;n<14;++n){struct pbuf *p=tcp_pbuf_prealloc(0,60,1460,&oversize,&pcb,1,1);assert(p->tot_len==60);}
 assert(allocated==14*EXPECTED_BYTES);
 printf("PASS pinned lwIP allocation: single=%u oversize=%u payload60 reserved=%u bytes/packet,14 packets=%u bytes\\n",LWIP_NETIF_TX_SINGLE_PBUF,TCP_OVERSIZE,EXPECTED_BYTES,allocated);return 0;}
''',encoding='utf-8')
cc=shlex.split(os.environ.get('CC','gcc'))
for single,oversize,expected in [(1,1460,1460),(0,0,60)]:
    exe=out/('tcp_allocator_'+str(single)+('.exe' if os.name=='nt' else ''))
    subprocess.run([*cc,'-std=c11','-Wall','-Wextra','-Werror','-O2','-UNDEBUG',
        '-DLWIP_NETIF_TX_SINGLE_PBUF='+str(single),'-DTCP_OVERSIZE='+str(oversize),
        '-DEXPECTED_BYTES='+str(expected),str(source),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
config=(ROOT/'lib/liblwip/sysdepend/pico_rp2040/lwipopts.h').read_text(encoding='utf-8')
assert '#define TCP_OVERSIZE                 0' in config
assert '#define LWIP_NETIF_TX_SINGLE_PBUF   0' in config
radio=(sdk/'lib/cyw43-driver/src/cyw43_ll.c').read_text(encoding='utf-8')
assert 'pbuf_copy_partial((const struct pbuf *)buf' in radio
print('PASS new options match the source; CYW43 upstream path supports pbuf chains via pbuf_copy_partial.')
