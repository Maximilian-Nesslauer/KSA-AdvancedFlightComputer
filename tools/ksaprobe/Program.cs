using System;
using System.Linq;
using System.Reflection;
using System.Reflection.Metadata;
using System.Reflection.PortableExecutable;

class SigProv : ISignatureTypeProvider<string, object>
{
    readonly MetadataReader R;
    public SigProv(MetadataReader r){R=r;}
    public string GetArrayType(string e, ArrayShape s)=>e+"[]";
    public string GetByReferenceType(string e)=>"ref "+e;
    public string GetFunctionPointerType(MethodSignature<string> s)=>"fnptr";
    public string GetGenericInstantiation(string g, System.Collections.Immutable.ImmutableArray<string> a)=>g+"<"+string.Join(",",a)+">";
    public string GetGenericMethodParameter(object c,int i)=>"!!"+i;
    public string GetGenericTypeParameter(object c,int i)=>"!"+i;
    public string GetModifiedType(string m,string u,bool req)=>u;
    public string GetPinnedType(string e)=>e;
    public string GetPointerType(string e)=>e+"*";
    public string GetPrimitiveType(PrimitiveTypeCode c)=>c.ToString();
    public string GetSZArrayType(string e)=>e+"[]";
    public string GetTypeFromDefinition(MetadataReader r,TypeDefinitionHandle h,byte b){var t=r.GetTypeDefinition(h);var ns=r.GetString(t.Namespace);var n=r.GetString(t.Name);return string.IsNullOrEmpty(ns)?n:ns+"."+n;}
    public string GetTypeFromReference(MetadataReader r,TypeReferenceHandle h,byte b){var t=r.GetTypeReference(h);var ns=r.GetString(t.Namespace);var n=r.GetString(t.Name);return string.IsNullOrEmpty(ns)?n:ns+"."+n;}
    public string GetTypeFromSpecification(MetadataReader r,object c,TypeSpecificationHandle h,byte b)=>r.GetTypeSpecification(h).DecodeSignature(this,c);
}

class Program
{
    static MetadataReader R;
    static string TypeName(TypeDefinition t) {
        var ns = R.GetString(t.Namespace);
        var n = R.GetString(t.Name);
        return string.IsNullOrEmpty(ns) ? n : ns + "." + n;
    }
    static void Main(string[] args)
    {
        string asm = @"C:\Program Files\Kitten Space Agency\KSA.dll";
        string mode = args.Length > 0 ? args[0] : "search";
        string term = args.Length > 1 ? args[1] : "";
        using var fs = System.IO.File.OpenRead(asm);
        using var pe = new PEReader(fs);
        R = pe.GetMetadataReader();

        if (mode == "members") {
            foreach (var th2 in R.TypeDefinitions) {
                var t = R.GetTypeDefinition(th2);
                if (TypeName(t) != term && R.GetString(t.Name) != term) continue;
                Console.WriteLine($"== TYPE {TypeName(t)}  ({t.Attributes & TypeAttributes.VisibilityMask}) ==");
                var sp = new SigProv(R);
                Console.WriteLine("-- methods --");
                foreach (var mh in t.GetMethods()) {
                    var m=R.GetMethodDefinition(mh); var sig=m.DecodeSignature(sp,null);
                    var pnames=new System.Collections.Generic.List<string>();
                    foreach (var ph in m.GetParameters()) {
                        var pd=R.GetParameter(ph);
                        if (pd.SequenceNumber==0) continue;
                        string dflt="";
                        var dc=pd.GetDefaultValue();
                        if (!dc.IsNil) { var c=R.GetConstant(dc); var br=R.GetBlobReader(c.Value); dflt=" = <default>"; }
                        bool isOut=(pd.Attributes & ParameterAttributes.Out)!=0;
                        while (pnames.Count < pd.SequenceNumber-1) pnames.Add("?");
                        pnames.Add((isOut?"out ":"")+R.GetString(pd.Name)+dflt);
                    }
                    var parts=new System.Collections.Generic.List<string>();
                    for (int i=0;i<sig.ParameterTypes.Length;i++)
                        parts.Add(sig.ParameterTypes[i]+" "+(i<pnames.Count?pnames[i]:"?"));
                    Console.WriteLine($"  {(m.Attributes&MethodAttributes.MemberAccessMask)} {((m.Attributes&MethodAttributes.Static)!=0?"static ":"")}{((m.Attributes&MethodAttributes.Virtual)!=0?"virtual ":"")}{sig.ReturnType} {R.GetString(m.Name)}({string.Join(", ", parts)})");
                }
                Console.WriteLine("-- fields --");
                foreach (var fh in t.GetFields()) { var f=R.GetFieldDefinition(fh); Console.WriteLine($"  {(f.Attributes&FieldAttributes.FieldAccessMask)} static={(f.Attributes&FieldAttributes.Static)!=0} {f.DecodeSignature(sp,null)} {R.GetString(f.Name)}"); }
                Console.WriteLine("-- events --");
                foreach (var eh in t.GetEvents()) { var e=R.GetEventDefinition(eh); Console.WriteLine($"  {R.GetString(e.Name)}"); }
                Console.WriteLine("-- nested --");
                foreach (var nh in t.GetNestedTypes()) { var n=R.GetTypeDefinition(nh); Console.WriteLine($"  {R.GetString(n.Name)}"); }
            }
            return;
        }

        foreach (var th in R.TypeDefinitions)
        {
            var t = R.GetTypeDefinition(th);
            string tn = TypeName(t);
            bool typeMatch = tn.IndexOf(term, StringComparison.OrdinalIgnoreCase) >= 0;

            // events
            foreach (var eh in t.GetEvents()) {
                var e = R.GetEventDefinition(eh);
                string en = R.GetString(e.Name);
                if (en.IndexOf(term, StringComparison.OrdinalIgnoreCase) >= 0)
                    Console.WriteLine($"EVENT  {tn}.{en}");
            }
            // methods
            foreach (var mh in t.GetMethods()) {
                var m = R.GetMethodDefinition(mh);
                string mnm = R.GetString(m.Name);
                if (mnm.IndexOf(term, StringComparison.OrdinalIgnoreCase) >= 0) {
                    var attr = m.Attributes;
                    string vis = (attr & MethodAttributes.Public)!=0 ? "pub" : (attr & MethodAttributes.Static)!=0?"":"";
                    bool isStatic = (attr & MethodAttributes.Static)!=0;
                    Console.WriteLine($"METHOD {tn}.{mnm}  static={isStatic} {(attr & MethodAttributes.MemberAccessMask)}");
                }
            }
            // fields
            foreach (var fh in t.GetFields()) {
                var f = R.GetFieldDefinition(fh);
                string fnm = R.GetString(f.Name);
                if (fnm.IndexOf(term, StringComparison.OrdinalIgnoreCase) >= 0)
                    Console.WriteLine($"FIELD  {tn}.{fnm}  static={(f.Attributes & FieldAttributes.Static)!=0} {(f.Attributes & FieldAttributes.FieldAccessMask)}");
            }
            if (typeMatch)
                Console.WriteLine($"TYPE   {tn}  ({t.Attributes & TypeAttributes.VisibilityMask})");
        }
    }
}
