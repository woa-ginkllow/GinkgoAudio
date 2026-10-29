/*
 * topo.cpp - filtro de topologia del altavoz.
 *
 * Dos pines: 0 = entrada desde el puente del wave, 1 = puente fisico hacia el
 * altavoz (KSNODETYPE_SPEAKER). Sin nodos: el volumen lo hace el motor de audio
 * de Windows por software; la limitacion de seguridad esta en wave.cpp.
 */
#include "ginkgoaud.h"

static KSDATARANGE g_TopoBridgeRange[] =
{
    {
        sizeof(KSDATARANGE), 0, 0, 0,
        STATICGUIDOF(KSDATAFORMAT_TYPE_AUDIO),
        STATICGUIDOF(KSDATAFORMAT_SUBTYPE_ANALOG),
        STATICGUIDOF(KSDATAFORMAT_SPECIFIER_NONE)
    }
};

static PKSDATARANGE g_TopoBridgeRangePtrs[] = { &g_TopoBridgeRange[0] };

static PCPIN_DESCRIPTOR g_TopoPins[] =
{
    {   /* 0: entrada (viene del puente del wave) */
        0, 0, 0, NULL,
        {
            0, NULL, 0, NULL,
            SIZEOF_ARRAY(g_TopoBridgeRangePtrs), g_TopoBridgeRangePtrs,
            KSPIN_DATAFLOW_IN, KSPIN_COMMUNICATION_NONE,
            &KSCATEGORY_AUDIO, NULL, 0
        }
    },
    {   /* 1: salida fisica = altavoz */
        0, 0, 0, NULL,
        {
            0, NULL, 0, NULL,
            SIZEOF_ARRAY(g_TopoBridgeRangePtrs), g_TopoBridgeRangePtrs,
            KSPIN_DATAFLOW_OUT, KSPIN_COMMUNICATION_NONE,
            &KSNODETYPE_SPEAKER, NULL, 0
        }
    }
};

static PCCONNECTION_DESCRIPTOR g_TopoConnections[] =
{
    { PCFILTER_NODE, 0, PCFILTER_NODE, 1 }
};

static PCFILTER_DESCRIPTOR g_TopoFilter =
{
    0,                                      /* Version */
    NULL,                                   /* AutomationTable */
    sizeof(PCPIN_DESCRIPTOR),
    SIZEOF_ARRAY(g_TopoPins),
    g_TopoPins,
    sizeof(PCNODE_DESCRIPTOR),
    0,
    NULL,
    SIZEOF_ARRAY(g_TopoConnections),
    g_TopoConnections,
    0,
    NULL
};

class CMiniportTopo : public IMiniportTopology, public CUnknown
{
public:
    DECLARE_STD_UNKNOWN();
    DEFINE_STD_CONSTRUCTOR(CMiniportTopo);
    ~CMiniportTopo() {}

    IMP_IMiniportTopology;
};

STDMETHODIMP_(NTSTATUS)
CMiniportTopo::NonDelegatingQueryInterface(_In_ REFIID Interface, _COM_Outptr_ PVOID *Object)
{
    if (IsEqualGUIDAligned(Interface, IID_IUnknown)) {
        *Object = PVOID(PUNKNOWN(PMINIPORTTOPOLOGY(this)));
    } else if (IsEqualGUIDAligned(Interface, IID_IMiniport)) {
        *Object = PVOID(PMINIPORT(this));
    } else if (IsEqualGUIDAligned(Interface, IID_IMiniportTopology)) {
        *Object = PVOID(PMINIPORTTOPOLOGY(this));
    } else {
        *Object = NULL;
    }

    if (*Object) {
        PUNKNOWN(*Object)->AddRef();
        return STATUS_SUCCESS;
    }
    return STATUS_INVALID_PARAMETER;
}

STDMETHODIMP_(NTSTATUS)
CMiniportTopo::GetDescription(_Out_ PPCFILTER_DESCRIPTOR *OutFilterDescriptor)
{
    *OutFilterDescriptor = &g_TopoFilter;
    return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportTopo::DataRangeIntersection(_In_ ULONG, _In_ PKSDATARANGE, _In_ PKSDATARANGE,
                                     _In_ ULONG, _Out_opt_ PVOID, _Out_ PULONG)
{
    return STATUS_NOT_IMPLEMENTED;
}

STDMETHODIMP_(NTSTATUS)
CMiniportTopo::Init(_In_ PUNKNOWN, _In_ PRESOURCELIST, _In_ PPORTTOPOLOGY)
{
    return STATUS_SUCCESS;
}

NTSTATUS CreateMiniportTopo(_Out_ PUNKNOWN *Unknown)
{
    CMiniportTopo *obj = new (POOL_FLAG_NON_PAGED, GKA_TAG) CMiniportTopo(NULL);
    if (!obj) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    obj->AddRef();
    *Unknown = static_cast<PUNKNOWN>(static_cast<PMINIPORTTOPOLOGY>(obj));
    return STATUS_SUCCESS;
}
