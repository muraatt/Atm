import {useId,useState,type ReactNode} from 'react';

export function PanelTabs({label,pages,initial}:{label:string;pages:{id:string;label:string;content:ReactNode}[];initial?:string}){
  const [selection,setSelection]=useState(initial??pages[0]?.id);
  const id=useId();
  const selected=pages.find(page=>page.id===selection)??pages[0];
  if(!selected)return null;
  return <div className="panel-pages">
    <div className="panel-tabbar" role="tablist" aria-label={label}>{pages.map((page,index)=><button key={page.id} id={`${id}-${page.id}`} role="tab" tabIndex={page.id===selected.id?0:-1} aria-controls={`${id}-panel`} aria-selected={page.id===selected.id} onClick={()=>setSelection(page.id)} onKeyDown={event=>{
      const next=event.key==='ArrowRight'?(index+1)%pages.length:event.key==='ArrowLeft'?(index+pages.length-1)%pages.length:event.key==='Home'?0:event.key==='End'?pages.length-1:null;
      if(next!==null){event.preventDefault();setSelection(pages[next].id);event.currentTarget.parentElement?.querySelectorAll<HTMLButtonElement>('[role=tab]')[next]?.focus();}
    }}>{page.label}</button>)}</div>
    <div id={`${id}-panel`} className={`panel-page page-${selected.id}`} role="tabpanel" aria-labelledby={`${id}-${selected.id}`}>{selected.content}</div>
  </div>;
}
