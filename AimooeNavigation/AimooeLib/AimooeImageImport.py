"""Background scalar-image conversion. Never connects to the optical device."""
import json
import sys
import traceback
from pathlib import Path
import SimpleITK as sitk


def convert(job):
    data=json.loads(job.read_text(encoding='utf-8'))
    source=Path(data['path'])
    readers=[]
    if source.is_dir():
        series=sitk.ImageSeriesReader.GetGDCMSeriesIDs(str(source))
        if not series:
            raise ValueError('目录中没有可读取的 DICOM 序列')
        for uid in series:
            files=sitk.ImageSeriesReader.GetGDCMSeriesFileNames(str(source),uid)
            reader=sitk.ImageSeriesReader()
            reader.SetFileNames(files)
            readers.append((uid,reader.Execute))
    else:
        readers.append((source.name,lambda: sitk.ReadImage(str(source))))
    result=[]
    for index,(name,read) in enumerate(readers):
        image=read()
        if image.GetDimension()!=3 or image.GetNumberOfComponentsPerPixel()!=1:
            raise ValueError('当前导航导入支持三维标量影像，请用 Slicer 原生导入器处理其他数据类型')
        target=job.parent/('volume-%d.nrrd'%index)
        sitk.WriteImage(image,str(target),False)
        result.append(dict(name=name,path=str(target)))
        del image
    return result


if __name__=='__main__':
    job=Path(sys.argv[1])
    try:
        result=dict(ok=True,volumes=convert(job))
    except Exception:
        result=dict(ok=False,error=traceback.format_exc())
    job.with_suffix('.result.json').write_text(json.dumps(result,ensure_ascii=False),encoding='utf-8')
